"""
    S/Space      : brake
    A/D          : steer left/right
    P            : toggle autopilot
    H            : spawn hazard

    TAB          : change sensor position
    [1-9]        : change to sensor [1-9]
    Backspace    : change vehicle

    ESC          : quit
"""

from __future__ import print_function


# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys
import zmq
import xlwings as xw
from time import time
import UdpComms as U

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/carla')
except IndexError:
    pass

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import carla

from carla import ColorConverter as cc

from agents.navigation.behavior_agent import BehaviorAgent  # pylint: disable=import-error
from agents.navigation.basic_agent import BasicAgent  # pylint: disable=import-error

import argparse
import collections
import datetime
import logging
import math
import random
import re
import weakref

if sys.version_info >= (3, 0):

    from configparser import ConfigParser

else:

    from ConfigParser import RawConfigParser as ConfigParser

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_0
    from pygame.locals import K_9
    from pygame.locals import K_BACKQUOTE
    from pygame.locals import K_BACKSPACE
    from pygame.locals import K_COMMA
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1
    from pygame.locals import K_LEFT
    from pygame.locals import K_PERIOD
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SLASH
    from pygame.locals import K_SPACE
    from pygame.locals import K_TAB
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_b
    from pygame.locals import K_c
    from pygame.locals import K_d
    from pygame.locals import K_g
    from pygame.locals import K_h
    from pygame.locals import K_i
    from pygame.locals import K_l
    from pygame.locals import K_m
    from pygame.locals import K_n
    from pygame.locals import K_o
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_t
    from pygame.locals import K_v
    from pygame.locals import K_w
    from pygame.locals import K_x
    from pygame.locals import K_z
    from pygame.locals import K_MINUS
    from pygame.locals import K_EQUALS
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')


# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================


def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name

def get_actor_blueprints(world, filter, generation):
    bps = world.get_blueprint_library().filter(filter)

    if generation.lower() == "all":
        return bps

    # If the filter returns only one bp, we assume that this one needed
    # and therefore, we ignore the generation
    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        # Check if generation is in available generations
        if int_generation in [1, 2]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except:
        print("   Warning! Actor Generation is not valid. No actor will be spawned.")
        return []

def get_angle(vector1, vector2):
    theta = math.acos(vector1.dot(vector2) / vector1.length() / vector2.length())
    return math.degrees(theta) if vector1.x * vector2.y - vector1.y * vector2.x > 0 else -math.degrees(theta)

axis_input = [0.0, 1.0]


# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================


class World(object):
    def __init__(self, carla_world, hud, args):
        self.world = carla_world
        self.sync = args.sync
        self.actor_role_name = args.rolename
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)
        self.hud = hud
        self.player = None
        self.destination = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.imu_sensor = None
        self.radar_sensor = None
        self.camera_manager = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self._actor_filter = args.filter
        self._actor_generation = args.generation
        self._gamma = args.gamma
        self.restart()
        self.world.on_tick(hud.on_world_tick)
        self.recording_enabled = False
        self.recording_start = 0
        self.constant_velocity_enabled = False
        self.show_vehicle_telemetry = False
        self.doors_are_open = False
        self.current_map_layer = 0
        self.map_layer_names = [
            carla.MapLayer.NONE,
            carla.MapLayer.Buildings,
            carla.MapLayer.Decals,
            carla.MapLayer.Foliage,
            carla.MapLayer.Ground,
            carla.MapLayer.ParkedVehicles,
            carla.MapLayer.Particles,
            carla.MapLayer.Props,
            carla.MapLayer.StreetLights,
            carla.MapLayer.Walls,
            carla.MapLayer.All
        ]
        self.imgl = pygame.image.load('bigl.png')
        self.imgl = pygame.transform.scale(self.imgl, (100, 100))
        self.imgr = pygame.image.load('bigr.png')
        self.imgr = pygame.transform.scale(self.imgr, (100, 100))

    def restart(self):
        self.player_max_speed = 1.589
        self.player_max_speed_fast = 3.713
        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0
        # Get a random blueprint.
        blueprint = random.choice(get_actor_blueprints(self.world, self._actor_filter, self._actor_generation))
        blueprint.set_attribute('role_name', self.actor_role_name)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        if blueprint.has_attribute('is_invincible'):
            blueprint.set_attribute('is_invincible', 'true')
        # set the max speed
        if blueprint.has_attribute('speed'):
            self.player_max_speed = float(blueprint.get_attribute('speed').recommended_values[1])
            self.player_max_speed_fast = float(blueprint.get_attribute('speed').recommended_values[2])

        # Spawn the player.
        if self.player is not None:
            # spawn_point = self.player.get_transform()
            spawn_point = random.choice(self.map.get_spawn_points())
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            self.show_vehicle_telemetry = False
            self.modify_vehicle_physics(self.player)
        while self.player is None:
            if not self.map.get_spawn_points():
                print('There are no spawn points available in your map/town.')
                print('Please add some Vehicle Spawn Point to your UE4 scene.')
                sys.exit(1)
            spawn_points = self.map.get_spawn_points()
            spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            self.show_vehicle_telemetry = False
            self.modify_vehicle_physics(self.player)
        # Set up the sensors.
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player)
        self.imu_sensor = IMUSensor(self.player)
        self.camera_manager = CameraManager(self.player, self.hud, self._gamma)
        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)
        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)

        if self.sync:
            self.world.tick()
        else:
            self.world.wait_for_tick()

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.player.get_world().set_weather(preset[0])

    def next_map_layer(self, reverse=False):
        self.current_map_layer += -1 if reverse else 1
        self.current_map_layer %= len(self.map_layer_names)
        selected = self.map_layer_names[self.current_map_layer]
        self.hud.notification('LayerMap selected: %s' % selected)

    def load_map_layer(self, unload=False):
        selected = self.map_layer_names[self.current_map_layer]
        if unload:
            self.hud.notification('Unloading map layer: %s' % selected)
            self.world.unload_map_layer(selected)
        else:
            self.hud.notification('Loading map layer: %s' % selected)
            self.world.load_map_layer(selected)

    def toggle_radar(self):
        if self.radar_sensor is None:
            self.radar_sensor = RadarSensor(self.player)
        elif self.radar_sensor.sensor is not None:
            self.radar_sensor.sensor.destroy()
            self.radar_sensor = None

    def modify_vehicle_physics(self, actor):
        #If actor is not a vehicle, we cannot use the physics control
        try:
            physics_control = actor.get_physics_control()
            physics_control.use_sweep_wheel_collision = True
            actor.apply_physics_control(physics_control)
        except Exception:
            pass

    def tick(self, clock):
        self.hud.tick(self, clock)

    def render(self, display):
        self.camera_manager.render(display)
        self.hud.render(display)
        display.blit(self.imgl, (1265, 0))
        display.blit(self.imgr, (2731, 0))

    def destroy_sensors(self):
        self.camera_manager.sensor.destroy()
        self.camera_manager.sensor = None
        self.camera_manager.index = None

    def destroy(self):
        if self.radar_sensor is not None:
            self.toggle_radar()
        sensors = [
            self.camera_manager.sensor,
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor,
            self.imu_sensor.sensor]
        for sensor in sensors:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()
        if self.player is not None:
            self.player.destroy()


# ==============================================================================
# -- HybridControl -------------------------------------------------------------
# ==============================================================================


class HybridControl(object):
    """Class that handles keyboard and joystick input."""
    def __init__(self, world, start_in_autopilot):
        self._autopilot_enabled = start_in_autopilot
        if isinstance(world.player, carla.Vehicle):
            self._control = carla.VehicleControl()
            self._lights = carla.VehicleLightState.NONE
            # world.player.set_autopilot(self._autopilot_enabled)
            world.player.set_light_state(self._lights)
        else:
            raise NotImplementedError("Actor type not supported")
        self._steer_cache = 0.0
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

        # initialize steering wheel
        pygame.joystick.init()

        joystick_count = pygame.joystick.get_count()
        if joystick_count > 1:
            raise ValueError("Please Connect Just One Joystick")

        self._joystick = pygame.joystick.Joystick(0)
        self._joystick.init()

        self._parser = ConfigParser()
        self._parser.read('wheel_config.ini')
        self._steer_idx = int(
            self._parser.get('G923 Racing Wheel', 'steering_wheel'))
        self._throttle_idx = int(
            self._parser.get('G923 Racing Wheel', 'throttle'))
        self._brake_idx = int(self._parser.get('G923 Racing Wheel', 'brake'))
        self._reverse_idx = int(self._parser.get('G923 Racing Wheel', 'reverse'))
        self._handbrake_idx = int(
            self._parser.get('G923 Racing Wheel', 'handbrake'))
        self._autopilot_idx = int(self._parser.get('G923 Racing Wheel', 'autopilot'))

    def parse_events(self, client, world, clock, sync_mode, agent, hazard):
        current_lights = self._lights
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True

            elif event.type == pygame.JOYBUTTONUP:
                # print(event.button)
                if event.button == self._reverse_idx:
                    self._control.gear = 1 if self._control.reverse else -1
                elif event.button == self._autopilot_idx:
                    self._autopilot_enabled = not self._autopilot_enabled
                    agent.set_destination(world.destination, True)

            elif event.type == pygame.JOYAXISMOTION:
                if event.axis == self._brake_idx:
                    axis_input[1] = event.value
                    if event.value < 0.9 and self._autopilot_enabled:
                        # throttle_cache = world.player.get_control().throttle
                        self._autopilot_enabled = False
                        # world.player.set_autopilot(False)
                        world.hud.notification(
                            'Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))
                        # self._control.throttle = throttle_cache
                elif event.axis == self._steer_idx:
                    axis_input[0] = event.value
                    if abs(event.value) > 0.01 and self._autopilot_enabled:
                        # throttle_cache = world.player.get_control().throttle
                        self._autopilot_enabled = False
                        # world.player.set_autopilot(False)
                        world.hud.notification(
                            'Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))
                        # self._control.throttle = throttle_cache
                        
            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_h:
                    world.hud.toggle_info()
                elif event.key == K_TAB:
                    world.camera_manager.toggle_camera()
                elif event.key > K_0 and event.key <= K_9:
                    index_ctrl = 0
                    if pygame.key.get_mods() & KMOD_CTRL:
                        index_ctrl = 9
                    world.camera_manager.set_sensor(event.key - 1 - K_0 + index_ctrl)
                elif event.key == K_q:
                    self._control.gear = 1 if self._control.reverse else -1
                elif event.key == K_p:
                    self._autopilot_enabled = not self._autopilot_enabled
                    agent.set_destination(world.destination, True)
                    world.hud.notification(
                        'Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))
                elif event.key == K_g:
                    transform = world.player.get_transform()
                    print('Location: %f %f %f, Rotation: %f' % (transform.location.x, transform.location.y, transform.location.z, transform.rotation.yaw))
                    
            elif event.type == pygame.KEYDOWN:
                if (event.key == K_a or event.key == K_d or event.key == K_SPACE or event.key == K_s) and self._autopilot_enabled:
                    throttle_cache = world.player.get_control().throttle
                    self._autopilot_enabled = False
                    world.hud.notification('Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))
                    self._control.throttle = throttle_cache

        if not self._autopilot_enabled:
            self._parse_vehicle_wheel()
            self._control.reverse = self._control.gear < 0
            # Set automatic control-related vehicle lights
            if self._control.brake:
                current_lights |= carla.VehicleLightState.Brake
            else: # Remove the Brake flag
                current_lights &= ~carla.VehicleLightState.Brake
            if self._control.reverse:
                current_lights |= carla.VehicleLightState.Reverse
            else: # Remove the Reverse flag
                current_lights &= ~carla.VehicleLightState.Reverse
            if current_lights != self._lights: # Change the light state only if necessary
                self._lights = current_lights
                world.player.set_light_state(carla.VehicleLightState(self._lights))
            world.player.apply_control(self._control)
        else:
            control = agent.run_step()
            control.manual_gear_shift = False
            world.player.apply_control(control)

    def _parse_vehicle_keys(self, keys, milliseconds):
        # if keys[K_UP] or keys[K_w]:
        #     self._control.throttle = min(self._control.throttle + 0.01, 1.00)
        # else:
        #     self._control.throttle = max(self._control.throttle - 0.01, 0.0)

        if keys[K_s]:
            self._control.brake = min(self._control.brake + 5e-4 * milliseconds, 1.0)
        else:
            self._control.brake = 0.0

        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            if self._steer_cache > 0:
                self._steer_cache = 0
            else:
                self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            if self._steer_cache < 0:
                self._steer_cache = 0
            else:
                self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)
        self._control.hand_brake = keys[K_SPACE]

    def _parse_vehicle_wheel(self):
        numAxes = self._joystick.get_numaxes()
        jsInputs = [float(self._joystick.get_axis(i)) for i in range(numAxes)]
        # print (jsInputs)
        jsButtons = [float(self._joystick.get_button(i)) for i in
                     range(self._joystick.get_numbuttons())]

        # Custom function to map range of inputs [1, -1] to outputs [0, 1] i.e 1 from inputs means nothing is pressed
        # For the steering, it seems fine as it is
        K1 = 0.4  # 0.55
        steerCmd = K1 * math.tan(1.1 * jsInputs[self._steer_idx])

        K2 = 1.6  # 1.6
        throttleCmd = K2 + (2.05 * math.log10(
            -0.7 * jsInputs[self._throttle_idx] + 1.4) - 1.2) / 0.92
        if throttleCmd <= 0:
            throttleCmd = 0
        elif throttleCmd > 1:
            throttleCmd = 1

        brakeCmd = 1.6 + (2.05 * math.log10(
            -0.7 * jsInputs[self._brake_idx]  + 1.4) - 1.2) / 0.92
        if brakeCmd <= 0:
            brakeCmd = 0
        elif brakeCmd > 1:
            brakeCmd = 1

        self._control.steer = steerCmd
        self._control.brake = brakeCmd / 2.0
        # self._control.throttle = throttleCmd
        if brakeCmd > 0.05:
            self._control.throttle = 0.0
        else:
            self._control.throttle = 0.5

        #toggle = jsButtons[self._reverse_idx]

        self._control.hand_brake = bool(jsButtons[self._handbrake_idx])

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)


# ==============================================================================
# -- HAZARD --------------------------------------------------------------------
# ==============================================================================


class Hazard(object):
    OVERT_PEDESTRIAN = 0
    OVERT_VEHICLE = 1
    COVERT_PEDESTRIAN = 2
    COVERT_VEHICLE = 3
    collision_trigger = False

    def __init__(self, start_points, end_points, hazard_transform):
        self.trigger = False
        self.active = False
        self.scene_active = False
        self.start_points = start_points
        self.end_points = end_points
        self.hazard_type = -1
        self.start_time = 0
        self.countdown = 0
        self.hazard_actors = []
        self.hazard_transform = hazard_transform
        self.hazard_vehicle = None
        self.hazard_angle = 0.0
        self.record = {
            "max_ax": 0.0,
            "max_ay": 0.0,
            "max_wheel": 0.0,
            "max_brake": 1.0,
            "collision": False,
            "min_col_distance": 0.0,
            "takeover_time": 0.0
        }
        self.startx = 0.0
        self.starty = 0.0

    def generate_hazard(self, world, agent, hazard_type):
        if self.active:
            return
        self.active = True
        self.hazard_type = hazard_type
        self.scene_point = self.hazard_transform[hazard_type]
        self.scene_point.location.z += 0.2
        agent.set_destination(self.start_points[hazard_type], True)
        world.destination = self.start_points[hazard_type]
        Hazard.collision_trigger = False

    def update(self, world, agent, autopilot):
        if self.hazard_vehicle is not None:
            if self.hazard_step == 0:
                self.hazard_vehicle.apply_control(carla.VehicleControl(throttle=0.2, steer=-0.35))
                self.hazard_step = 1
            elif self.hazard_step == 1 and self.start_time + 4.2 < time():
                self.hazard_vehicle.apply_control(carla.VehicleControl(throttle=0.1, steer=0.25))
                self.hazard_step = 2
            elif self.hazard_step == 2 and self.start_time + 7 < time():
                self.hazard_vehicle.apply_control(carla.VehicleControl(throttle=0.1, steer=0))
                self.hazard_step = 3

            # if self.hazard_vehicle.get_location().distance(world.player.get_location()) > 160.0:
        if not self.active:
            return
        if agent.done():
            if not self.scene_active:
                player_location = world.player.get_location()
                self.startx = player_location.x
                self.starty = player_location.y
                actors = world.world.get_actors().filter('vehicle.*')
                for actor in actors:
                    if actor.id == world.player.id:
                        continue
                    actor_location = actor.get_location()
                    if (abs(actor_location.x - self.startx) < 5.0
                        and actor_location.y - self.starty > 0
                        and actor_location.y - self.starty < self.end_points[self.hazard_type].y - self.starty + 5.0):
                        actor.destroy()

                agent.ignore_vehicles(True)
                agent.set_destination(self.end_points[self.hazard_type])

                self.scene_active = True
                self.start_time = time()
                self.countdown = 30
                self.trigger = True

                if self.hazard_type == Hazard.OVERT_PEDESTRIAN:
                    hazard_bp = world.world.get_blueprint_library().filter('pedestrian')[0]
                    if hazard_bp.has_attribute('is_invincible'):
                        hazard_bp.set_attribute('is_invincible', 'false')
                    hazard_transform = self.scene_point
                    hazard_transform.rotation.yaw -= 90.0
                    hazard_transform.location += -3.0 * hazard_transform.get_forward_vector()
                    hazard_transform.location.z += 0.5
                    hazard_walker = world.world.spawn_actor(hazard_bp, hazard_transform)
                    hazard_walker.apply_control(carla.WalkerControl(speed=0.8))
                    self.hazard_actors.append(hazard_walker)

                elif self.hazard_type == Hazard.OVERT_VEHICLE:
                    hazard_bp = world.world.get_blueprint_library().filter('model3')[0]
                    hazard_bp.set_attribute('color', '0,0,0')
                    hazard_transform = self.scene_point
                    hazard_vehicle = world.world.spawn_actor(hazard_bp, hazard_transform)
                    hazard_vehicle.apply_control(carla.VehicleControl(throttle=0.5, steer=0.0, brake=1.0, hand_brake=True))
                    self.hazard_actors.append(hazard_vehicle)

                    hazard_bp = world.world.get_blueprint_library().filter('trafficcone01')[0]
                    hazard_transform.location += -5.0 * hazard_transform.get_forward_vector()
                    hazard_transform.location.z -= 0.5
                    hazard_obstacle = world.world.spawn_actor(hazard_bp, hazard_transform)
                    self.hazard_actors.append(hazard_obstacle)

                elif self.hazard_type == Hazard.COVERT_PEDESTRIAN:
                    hazard_bp = world.world.get_blueprint_library().filter('pedestrian')[0]
                    if hazard_bp.has_attribute('is_invincible'):
                        hazard_bp.set_attribute('is_invincible', 'false')
                    hazard_transform = self.scene_point
                    obstacle_transform = carla.Transform(self.scene_point.location, self.scene_point.rotation)
                    hazard_transform.rotation.yaw -= 90.0
                    # hazard_transform.location += -3.0 * hazard_transform.get_forward_vector()
                    hazard_transform.location.z += 0.3
                    hazard_walker = world.world.spawn_actor(hazard_bp, hazard_transform)
                    hazard_walker.apply_control(carla.WalkerControl(speed=0.8))
                    self.hazard_actors.append(hazard_walker)
                    
                    hazard_bp = world.world.get_blueprint_library().filter('ambulance')[0]
                    obstacle_transform.location += -4.0 * obstacle_transform.get_forward_vector()
                    obstacle_transform.location.z -= 0.2
                    hazard_obstacle = world.world.spawn_actor(hazard_bp, obstacle_transform)
                    self.hazard_actors.append(hazard_obstacle)

                elif self.hazard_type == Hazard.COVERT_VEHICLE:
                    if self.hazard_vehicle is not None:
                        self.hazard_vehicle.destroy()
                        self.hazard_vehicle = None
                    hazard_bp = world.world.get_blueprint_library().filter('model3')[0]
                    hazard_bp.set_attribute('color', '0,0,0')
                    hazard_transform = self.scene_point
                    hazard_transform.location.z -= 0.2
                    self.hazard_vehicle = world.world.spawn_actor(hazard_bp, hazard_transform)
                    self.hazard_destination = hazard_transform.location + carla.Vector3D(x=3.2, y=20.0)
                    self.hazard_step = 0

                    hazard_bp = world.world.get_blueprint_library().filter('ambulance')[0]
                    hazard_transform.location += -6.0 * hazard_transform.get_forward_vector()
                    hazard_transform.location.z += 0.6
                    hazard_obstacle = world.world.spawn_actor(hazard_bp, hazard_transform)
                    hazard_obstacle.apply_control(carla.VehicleControl(hand_brake=True))
                    self.hazard_actors.append(hazard_obstacle)
            else:
                agent.ignore_vehicles(False)
                world.destination = random.choice(world.map.get_spawn_points()).location
                self.reset()
                return True
        if self.scene_active:
            current_time = time()

            if current_time < self.start_time + 10:
                if Hazard.collision_trigger:
                    self.record['collision'] = True
                new_ax = world.imu_sensor.accelerometer[0]
                new_ay = world.imu_sensor.accelerometer[1]
                if -new_ax > abs(self.record['max_ax']) and world.player.get_velocity().length() > 18.0 / 3.6:
                    self.record['max_ax'] = new_ax
                if abs(new_ay) > abs(self.record['max_ay']) and world.player.get_velocity().length() > 18.0 / 3.6:
                    self.record['max_ay'] = new_ay
                if abs(axis_input[0]) > abs(self.record['max_wheel']):
                    self.record['max_wheel'] = axis_input[0]
                if axis_input[1] < abs(self.record['max_brake']):
                    self.record['max_brake'] = axis_input[1]
                if self.record['min_col_distance'] == 0.0 and (world.player.get_velocity().length() < 0.2 or abs(world.player.get_location().x - self.startx) > 2.0):
                    hazard_location = self.hazard_actors[0].get_location() if self.hazard_type != self.COVERT_VEHICLE else self.hazard_vehicle.get_location()
                    self.record['min_col_distance'] = world.player.get_location().distance_2d(hazard_location)
                if not autopilot and self.record['takeover_time'] == 0.0:
                    self.record['takeover_time'] = current_time - self.start_time

            hazard_location = self.hazard_actors[0].get_location() if self.hazard_type != self.COVERT_VEHICLE else self.hazard_vehicle.get_location()
            self.hazard_angle = get_angle(world.player.get_transform().get_forward_vector(), hazard_location - world.player.get_location())
            if self.start_time + self.countdown < current_time:
                agent.ignore_vehicles(False)
                world.destination = random.choice(world.map.get_spawn_points()).location
                self.reset()
                return True


        
        Hazard.collision_trigger = False

    def reset(self, is_record=True):
        if is_record:
            # wb = xw.Book('data_basic.xlsx')
            wb = xw.Book('data_direction.xlsx')
            # wb = xw.Book('data_time.xlsx')
            if self.hazard_type == self.OVERT_PEDESTRIAN:
                sht = wb.sheets['Overt Pedestrian']
            if self.hazard_type == self.OVERT_VEHICLE:
                sht = wb.sheets['Overt Vehicle']
            if self.hazard_type == self.COVERT_PEDESTRIAN:
                sht = wb.sheets['Covert Pedestrian']
            if self.hazard_type == self.COVERT_VEHICLE:
                sht = wb.sheets['Covert Vehicle']
            start_row = 2
            while sht.range('B' + start_row.__str__()).value is not None:
                start_row += 1
            start_row = start_row.__str__()
            sht.range('B' + start_row).value = self.record['max_ax']
            sht.range('C' + start_row).value = self.record['max_ay']
            sht.range('D' + start_row).value = self.record['max_wheel']
            sht.range('E' + start_row).value = self.record['max_brake'] / 2.0 + 0.5
            sht.range('F' + start_row).value = 1 if self.record['collision'] else 0
            sht.range('G' + start_row).value = self.record['min_col_distance']
            sht.range('H' + start_row).value = self.record['takeover_time']

        self.trigger = False
        self.active = False
        self.scene_active = False
        self.hazard_type = -1
        self.start_time = 0.0
        self.countdown = 0
        for actor in self.hazard_actors:
            actor.destroy()
        if self.hazard_vehicle is not None:
            self.hazard_vehicle.destroy()
            self.hazard_vehicle = None
        self.hazard_actors.clear()
        self.hazard_angle = 0.0
        self.record['max_ax'] = 0.0
        self.record['max_ay'] = 0.0
        self.record['max_wheel'] = 0.0
        self.record['max_brake'] = 1.0
        self.record['collision'] = False
        self.record['min_col_distance'] = 0.0
        self.record['takeover_time'] = 0.0
        self.startx = 0.0
        self.starty = 0.0


# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================


class HUD(object):
    def __init__(self, width, height):
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        font_name = 'courier' if os.name == 'nt' else 'mono'
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)
        self._notifications = FadingText(font, (width, 40), (0, height - 40))
        self.help = HelpText(pygame.font.Font(mono, 16), width, height)
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
        self._show_info = True
        self._info_text = []
        self._server_clock = pygame.time.Clock()

    def on_world_tick(self, timestamp):
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame
        self.simulation_time = timestamp.elapsed_seconds

    def tick(self, world, clock):
        self._notifications.tick(world, clock)
        if not self._show_info:
            return
        t = world.player.get_transform()
        v = world.player.get_velocity()
        c = world.player.get_control()
        compass = world.imu_sensor.compass
        heading = 'N' if compass > 270.5 or compass < 89.5 else ''
        heading += 'S' if 90.5 < compass < 269.5 else ''
        heading += 'E' if 0.5 < compass < 179.5 else ''
        heading += 'W' if 180.5 < compass < 359.5 else ''
        colhist = world.collision_sensor.get_collision_history()
        collision = [colhist[x + self.frame - 200] for x in range(0, 200)]
        max_col = max(1.0, max(collision))
        collision = [x / max_col for x in collision]
        vehicles = world.world.get_actors().filter('vehicle.*')
        self._info_text = [
            'Server:  % 16.0f FPS' % self.server_fps,
            'Client:  % 16.0f FPS' % clock.get_fps(),
            '',
            'Vehicle: % 20s' % get_actor_display_name(world.player, truncate=20),
            'Map:     % 20s' % world.map.name.split('/')[-1],
            'Simulation time: % 12s' % datetime.timedelta(seconds=int(self.simulation_time)),
            '',
            'Speed:   % 15.0f km/h' % (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)),
            u'Compass:% 17.0f\N{DEGREE SIGN} % 2s' % (compass, heading),
            'Accelero: (%5.1f,%5.1f,%5.1f)' % (world.imu_sensor.accelerometer),
            'Gyroscop: (%5.1f,%5.1f,%5.1f)' % (world.imu_sensor.gyroscope),
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (t.location.x, t.location.y)),
            'GNSS:% 24s' % ('(% 2.6f, % 3.6f)' % (world.gnss_sensor.lat, world.gnss_sensor.lon)),
            'Height:  % 18.0f m' % t.location.z,
            '']
        if isinstance(c, carla.VehicleControl):
            self._info_text += [
                ('Throttle:', c.throttle, 0.0, 1.0),
                ('Steer:', c.steer, -1.0, 1.0),
                ('Brake:', c.brake, 0.0, 1.0),
                ('Reverse:', c.reverse),
                ('Hand brake:', c.hand_brake),
                ('Manual:', c.manual_gear_shift),
                'Gear:        %s' % {-1: 'R', 0: 'N'}.get(c.gear, c.gear)]
        elif isinstance(c, carla.WalkerControl):
            self._info_text += [
                ('Speed:', c.speed, 0.0, 5.556),
                ('Jump:', c.jump)]
        self._info_text += [
            '',
            'Collision:',
            collision,
            '',
            'Number of vehicles: % 8d' % len(vehicles)]
        if len(vehicles) > 1:
            self._info_text += ['Nearby vehicles:']
            distance = lambda l: math.sqrt((l.x - t.location.x)**2 + (l.y - t.location.y)**2 + (l.z - t.location.z)**2)
            vehicles = [(distance(x.get_location()), x) for x in vehicles if x.id != world.player.id]
            for d, vehicle in sorted(vehicles, key=lambda vehicles: vehicles[0]):
                if d > 200.0:
                    break
                vehicle_type = get_actor_display_name(vehicle, truncate=22)
                self._info_text.append('% 4dm %s' % (d, vehicle_type))

    def toggle_info(self):
        self._show_info = not self._show_info

    def notification(self, text, seconds=2.0):
        # self._notifications.set_text(text, seconds=seconds)
        None

    def error(self, text):
        self._notifications.set_text('Error: %s' % text, (255, 0, 0))

    def render(self, display):
        if self._show_info:
            info_surface = pygame.Surface((220, self.dim[1]))
            info_surface.set_alpha(100)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1.0 - y) * 30) for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        f = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect((bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            rect = pygame.Rect((bar_h_offset, v_offset + 8), (f * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item:  # At this point has to be a str.
                    surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
        self._notifications.render(display)
        self.help.render(display)


# ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================


class FadingText(object):
    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, _, clock):
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        display.blit(self.surface, self.pos)


# ==============================================================================
# -- HelpText ------------------------------------------------------------------
# ==============================================================================


class HelpText(object):
    """Helper class to handle text output using pygame"""
    def __init__(self, font, width, height):
        lines = __doc__.split('\n')
        self.font = font
        self.line_space = 18
        self.dim = (780, len(lines) * self.line_space + 12)
        self.pos = (0.5 * width - 0.5 * self.dim[0], 0.5 * height - 0.5 * self.dim[1])
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)
        self.surface.fill((0, 0, 0, 0))
        for n, line in enumerate(lines):
            text_texture = self.font.render(line, True, (255, 255, 255))
            self.surface.blit(text_texture, (22, n * self.line_space))
            self._render = False
        self.surface.set_alpha(220)

    def toggle(self):
        self._render = not self._render

    def render(self, display):
        if self._render:
            display.blit(self.surface, self.pos)


# ==============================================================================
# -- CollisionSensor -----------------------------------------------------------
# ==============================================================================


class CollisionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    def get_collision_history(self):
        history = collections.defaultdict(int)
        for frame, intensity in self.history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        self.hud.notification('Collision with %r' % actor_type)
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)
        Hazard.collision_trigger = True


# ==============================================================================
# -- LaneInvasionSensor --------------------------------------------------------
# ==============================================================================


class LaneInvasionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None

        # If the spawn object is not a vehicle, we cannot use the Lane Invasion Sensor
        if parent_actor.type_id.startswith("vehicle."):
            self._parent = parent_actor
            self.hud = hud
            world = self._parent.get_world()
            bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
            self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
            # We need to pass the lambda a weak reference to self to avoid circular
            # reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda event: LaneInvasionSensor._on_invasion(weak_self, event))

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return
        lane_types = set(x.type for x in event.crossed_lane_markings)
        text = ['%r' % str(x).split()[-1] for x in lane_types]
        self.hud.notification('Crossed line %s' % ' and '.join(text))


# ==============================================================================
# -- GnssSensor ----------------------------------------------------------------
# ==============================================================================


class GnssSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.lat = 0.0
        self.lon = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.gnss')
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.0, z=2.8)), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))

    @staticmethod
    def _on_gnss_event(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.lat = event.latitude
        self.lon = event.longitude


# ==============================================================================
# -- IMUSensor -----------------------------------------------------------------
# ==============================================================================


class IMUSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.accelerometer = (0.0, 0.0, 0.0)
        self.gyroscope = (0.0, 0.0, 0.0)
        self.compass = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.imu')
        self.sensor = world.spawn_actor(
            bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda sensor_data: IMUSensor._IMU_callback(weak_self, sensor_data))

    @staticmethod
    def _IMU_callback(weak_self, sensor_data):
        self = weak_self()
        if not self:
            return
        limits = (-99.9, 99.9)
        self.accelerometer = (
            max(limits[0], min(limits[1], sensor_data.accelerometer.x)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.y)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.z)))
        self.gyroscope = (
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.x))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.y))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.z))))
        self.compass = math.degrees(sensor_data.compass)


# ==============================================================================
# -- RadarSensor ---------------------------------------------------------------
# ==============================================================================


class RadarSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        bound_x = 0.5 + self._parent.bounding_box.extent.x
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        bound_z = 0.5 + self._parent.bounding_box.extent.z

        self.velocity_range = 7.5 # m/s
        world = self._parent.get_world()
        self.debug = world.debug
        bp = world.get_blueprint_library().find('sensor.other.radar')
        bp.set_attribute('horizontal_fov', str(35))
        bp.set_attribute('vertical_fov', str(20))
        self.sensor = world.spawn_actor(
            bp,
            carla.Transform(
                carla.Location(x=bound_x + 0.05, z=bound_z+0.05),
                carla.Rotation(pitch=5)),
            attach_to=self._parent)
        # We need a weak reference to self to avoid circular reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda radar_data: RadarSensor._Radar_callback(weak_self, radar_data))

    @staticmethod
    def _Radar_callback(weak_self, radar_data):
        self = weak_self()
        if not self:
            return
        # To get a numpy [[vel, altitude, azimuth, depth],...[,,,]]:
        # points = np.frombuffer(radar_data.raw_data, dtype=np.dtype('f4'))
        # points = np.reshape(points, (len(radar_data), 4))

        current_rot = radar_data.transform.rotation
        for detect in radar_data:
            azi = math.degrees(detect.azimuth)
            alt = math.degrees(detect.altitude)
            # The 0.25 adjusts a bit the distance so the dots can
            # be properly seen
            fw_vec = carla.Vector3D(x=detect.depth - 0.25)
            carla.Transform(
                carla.Location(),
                carla.Rotation(
                    pitch=current_rot.pitch + alt,
                    yaw=current_rot.yaw + azi,
                    roll=current_rot.roll)).transform(fw_vec)

            def clamp(min_v, max_v, value):
                return max(min_v, min(value, max_v))

            norm_velocity = detect.velocity / self.velocity_range # range [-1, 1]
            r = int(clamp(0.0, 1.0, 1.0 - norm_velocity) * 255.0)
            g = int(clamp(0.0, 1.0, 1.0 - abs(norm_velocity)) * 255.0)
            b = int(abs(clamp(- 1.0, 0.0, - 1.0 - norm_velocity)) * 255.0)
            self.debug.draw_point(
                radar_data.transform.location + fw_vec,
                size=0.075,
                life_time=0.06,
                persistent_lines=False,
                color=carla.Color(r, g, b))

# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================


class CameraManager(object):
    def __init__(self, parent_actor, hud, gamma_correction):
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.hud = hud
        self.recording = False
        bound_x = 0.5 + self._parent.bounding_box.extent.x
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        bound_z = 0.5 + self._parent.bounding_box.extent.z
        Attachment = carla.AttachmentType

        if not self._parent.type_id.startswith("walker.pedestrian"):
            self._camera_transforms = [
                (carla.Transform(carla.Location(x=-2.0*bound_x, y=+0.0*bound_y, z=2.0*bound_z), carla.Rotation(pitch=8.0)), Attachment.SpringArm),
                (carla.Transform(carla.Location(x=+0.8*bound_x, y=+0.0*bound_y, z=1.3*bound_z)), Attachment.Rigid),
                (carla.Transform(carla.Location(x=+1.9*bound_x, y=+1.0*bound_y, z=1.2*bound_z)), Attachment.SpringArm),
                (carla.Transform(carla.Location(x=-2.8*bound_x, y=+0.0*bound_y, z=4.6*bound_z), carla.Rotation(pitch=6.0)), Attachment.SpringArm),
                (carla.Transform(carla.Location(x=-1.0, y=-1.0*bound_y, z=0.4*bound_z)), Attachment.Rigid)]
        else:
            self._camera_transforms = [
                (carla.Transform(carla.Location(x=-2.5, z=0.0), carla.Rotation(pitch=-8.0)), Attachment.SpringArm),
                (carla.Transform(carla.Location(x=1.6, z=1.7)), Attachment.Rigid),
                (carla.Transform(carla.Location(x=2.5, y=0.5, z=0.0), carla.Rotation(pitch=-8.0)), Attachment.SpringArm),
                (carla.Transform(carla.Location(x=-4.0, z=2.0), carla.Rotation(pitch=6.0)), Attachment.SpringArm),
                (carla.Transform(carla.Location(x=0, y=-2.5, z=-0.0), carla.Rotation(yaw=90.0)), Attachment.Rigid)]

        self.transform_index = 1
        self.sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB', {}],
            ['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)', {}],
            ['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)', {}],
            ['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)', {}],
            ['sensor.camera.semantic_segmentation', cc.Raw, 'Camera Semantic Segmentation (Raw)', {}],
            ['sensor.camera.semantic_segmentation', cc.CityScapesPalette, 'Camera Semantic Segmentation (CityScapes Palette)', {}],
            ['sensor.camera.instance_segmentation', cc.CityScapesPalette, 'Camera Instance Segmentation (CityScapes Palette)', {}],
            ['sensor.camera.instance_segmentation', cc.Raw, 'Camera Instance Segmentation (Raw)', {}],
            ['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)', {'range': '50'}],
            ['sensor.camera.dvs', cc.Raw, 'Dynamic Vision Sensor', {}],
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB Distorted',
                {'lens_circle_multiplier': '3.0',
                'lens_circle_falloff': '3.0',
                'chromatic_aberration_intensity': '0.5',
                'chromatic_aberration_offset': '0'}],
            ['sensor.camera.optical_flow', cc.Raw, 'Optical Flow', {}],
        ]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(hud.dim[0]))
                bp.set_attribute('image_size_y', str(hud.dim[1]))
                bp.set_attribute('fov', str(120))
                if bp.has_attribute('gamma'):
                    bp.set_attribute('gamma', str(gamma_correction))
                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)
            elif item[0].startswith('sensor.lidar'):
                self.lidar_range = 50

                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)
                    if attr_name == 'range':
                        self.lidar_range = float(attr_value)

            item.append(bp)
        self.index = None

    def toggle_camera(self):
        self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        self.set_sensor(self.index, notify=False, force_respawn=True)

    def set_sensor(self, index, notify=True, force_respawn=False):
        index = index % len(self.sensors)
        needs_respawn = True if self.index is None else \
            (force_respawn or (self.sensors[index][2] != self.sensors[self.index][2]))
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self.sensors[index][-1],
                self._camera_transforms[self.transform_index][0],
                attach_to=self._parent,
                attachment_type=self._camera_transforms[self.transform_index][1])
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        if notify:
            self.hud.notification(self.sensors[index][2])
        self.index = index

    def next_sensor(self):
        self.set_sensor(self.index + 1)

    def toggle_recording(self):
        self.recording = not self.recording
        self.hud.notification('Recording %s' % ('On' if self.recording else 'Off'))

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))
            # pygame.transform.scale(self.surface, (7680, 1440), display)
            # pygame.transform.scale2x(self.surface, display)

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        if self.sensors[self.index][0].startswith('sensor.lidar'):
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 4), 4))
            lidar_data = np.array(points[:, :2])
            lidar_data *= min(self.hud.dim) / (2.0 * self.lidar_range)
            lidar_data += (0.5 * self.hud.dim[0], 0.5 * self.hud.dim[1])
            lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (self.hud.dim[0], self.hud.dim[1], 3)
            lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            self.surface = pygame.surfarray.make_surface(lidar_img)
        elif self.sensors[self.index][0].startswith('sensor.camera.dvs'):
            # Example of converting the raw_data from a carla.DVSEventArray
            # sensor into a NumPy array and using it as an image
            dvs_events = np.frombuffer(image.raw_data, dtype=np.dtype([
                ('x', np.uint16), ('y', np.uint16), ('t', np.int64), ('pol', np.bool)]))
            dvs_img = np.zeros((image.height, image.width, 3), dtype=np.uint8)
            # Blue is positive, red is negative
            dvs_img[dvs_events[:]['y'], dvs_events[:]['x'], dvs_events[:]['pol'] * 2] = 255
            self.surface = pygame.surfarray.make_surface(dvs_img.swapaxes(0, 1))
        elif self.sensors[self.index][0].startswith('sensor.camera.optical_flow'):
            image = image.get_color_coded_flow()
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        else:
            image.convert(self.sensors[self.index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self.recording:
            image.save_to_disk('_out/%08d' % image.frame)


# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================


def game_loop(args):
    pygame.init()
    pygame.font.init()
    world = None
    original_settings = None

    # try:
    client = carla.Client(args.host, args.port)
    client.set_timeout(20.0)

    sim_world = client.get_world()
    if args.sync:
        original_settings = sim_world.get_settings()
        settings = sim_world.get_settings()
        if not settings.synchronous_mode:
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.02
        sim_world.apply_settings(settings)

        traffic_manager = client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)

    if args.autopilot and not sim_world.get_settings().synchronous_mode:
        print("WARNING: You are currently in asynchronous mode and could "
                "experience some issues with the traffic simulation")

    display_flags = pygame.HWSURFACE | pygame.DOUBLEBUF
    if args.fullscreen:
        display_flags |= pygame.FULLSCREEN
    display = pygame.display.set_mode(
        (args.width, args.height),
        display_flags)
    display.fill((0,0,0))
    pygame.display.flip()

    hud = HUD(args.width, args.height)
    world = World(sim_world, hud, args)
    controller = HybridControl(world, args.autopilot)
    agent = BasicAgent(world.player, 30, {'ignore_traffic_lights': True})
    
    spawn_points = world.map.get_spawn_points()
    world.destination = random.choice(spawn_points).location
    agent.set_destination(world.destination, True)
    
    hazard = Hazard({
            Hazard.OVERT_PEDESTRIAN: world.map.get_waypoint(carla.Location(x=-88.3, y=60.0)).transform.location,
            Hazard.OVERT_VEHICLE: world.map.get_waypoint(carla.Location(x=-88.3, y=40.0)).transform.location,
            Hazard.COVERT_PEDESTRIAN: world.map.get_waypoint(carla.Location(x=-85.3, y=60.0)).transform.location,
            Hazard.COVERT_VEHICLE: world.map.get_waypoint(carla.Location(x=-88.3, y=40.0)).transform.location
        },
        {
            Hazard.OVERT_PEDESTRIAN: world.map.get_waypoint(carla.Location(x=-88.2, y=159.0)).transform.location,
            Hazard.OVERT_VEHICLE: world.map.get_waypoint(carla.Location(x=-88.2, y=159.0)).transform.location,
            Hazard.COVERT_PEDESTRIAN: world.map.get_waypoint(carla.Location(x=-85.3, y=159.0)).transform.location,
            Hazard.COVERT_VEHICLE: world.map.get_waypoint(carla.Location(x=-88.2, y=159.0)).transform.location,
        },
        {
            Hazard.OVERT_PEDESTRIAN: carla.Transform(carla.Location(x=-91.0, y=123.2, z=0.5), carla.Rotation(yaw=89.0)),
            Hazard.OVERT_VEHICLE: carla.Transform(carla.Location(x=-88.3, y=100.0, z=0.5), carla.Rotation(yaw=89.0)),
            Hazard.COVERT_PEDESTRIAN: carla.Transform(carla.Location(x=-91.0, y=123.2, z=0.5), carla.Rotation(yaw=89.0)),
            Hazard.COVERT_VEHICLE: carla.Transform(carla.Location(x=-91.0, y=95.0, z=0.5), carla.Rotation(yaw=89.0))
        }
    )

    clock = pygame.time.Clock()

    last_hazard_time = time()
    hazard_gap = 40
    hazard_list = [Hazard.OVERT_PEDESTRIAN, Hazard.OVERT_VEHICLE, Hazard.COVERT_PEDESTRIAN, Hazard.COVERT_VEHICLE]
    # hazard_list = [Hazard.OVERT_VEHICLE, Hazard.COVERT_VEHICLE]
    random.shuffle(hazard_list)
    hazard_prog = 0

    sock = U.UdpComms(udpIP="127.0.0.1", portTX=8000, portRX=8001, enableRX=False, suppressWarnings=True)

    while True:
        clock.tick()
        if args.sync:
            sim_world.tick()
        else:
            sim_world.wait_for_tick()
        current_time = time()
        if current_time > last_hazard_time + hazard_gap:
            print(hazard_prog)
            if hazard_prog > 3:
                return
            hazard.generate_hazard(world, agent, hazard_list[hazard_prog])
            # last_hazard_time = current_time
            last_hazard_time += 3600
            hazard_prog += 1
        
        if hazard.update(world, agent, controller._autopilot_enabled):
            last_hazard_time = current_time
        if controller.parse_events(client, world, clock, args.sync, agent, hazard):
            return
        world.tick(clock)
        world.render(display)

        pygame.display.flip()

        if agent.done() and controller._autopilot_enabled and not hazard.active:
            world.destination = random.choice(spawn_points).location
            agent.set_destination(world.destination, True)
        
        info = 'velocity=' + round(world.player.get_velocity().length() * 3.6).__str__()
        if hazard.trigger:
            info += ',trigger'
            hazard.trigger = False
        if hazard.scene_active:
            info += ',direction=' + (hazard.hazard_angle * -5).__str__()
        info += ',automode=' + ('1' if controller._autopilot_enabled else '0')
        info += ',available=' + ('1' if not controller._autopilot_enabled and not hazard.scene_active else '0')
        sock.SendData(info)

    # finally:
    hazard.reset(False)

    if original_settings:
        sim_world.apply_settings(original_settings)

    if (world and world.recording_enabled):
        client.stop_recorder()

    if world is not None:
        world.destroy()

    pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '--fullscreen',
        action='store_true',
        help='enable full screen display')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='4096x768',
        # default='7680x1440',
        help='window resolution (default: 1280x720)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.audi.tt',
        help='actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '--generation',
        metavar='G',
        default='2',
        help='restrict to certain actor generation (values: "1","2","All" - default: "2")')
    argparser.add_argument(
        '--rolename',
        metavar='NAME',
        default='hero',
        help='actor role name (default: "hero")')
    argparser.add_argument(
        '--gamma',
        default=2.2,
        type=float,
        help='Gamma correction of the camera (default: 2.2)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Activate synchronous mode execution')
    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    try:

        game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':

    main()
