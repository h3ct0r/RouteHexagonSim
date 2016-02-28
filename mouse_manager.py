from math import sqrt
import buttons
import math_helper
import pygame
import config
import simulation
import os
import json
from shapely.geometry import *

__author__ = 'Hector Azpurua'

pygame.init()


class MouseManager(object):

    def __init__(self, simulator):
        self.sim = simulator

        self.button_up = buttons.Button()
        self.button_down = buttons.Button()

        self.button_more_size = buttons.Button()
        self.button_less_size = buttons.Button()

        self.button_more_battery = buttons.Button()
        self.button_less_battery = buttons.Button()

        self.button_bigger_width_lines = buttons.Button()
        self.button_less_width_lines = buttons.Button()

        self.button_change_direction = buttons.Button()

        self.button_change_division_algorithm = buttons.Button()

        self.button_load_saved_points = buttons.Button()
        self.button_save_points = buttons.Button()

        self.button_more_height = buttons.Button()
        self.button_less_height = buttons.Button()

        self.draw_buttons()
        pass

    def draw_buttons(self):
        self.button_less_width_lines.create_button(
            self.sim.screen, (107, 142, 35), 0, 0, 50, 40, 0, "-WIDTH", (255, 255, 255))
        self.button_bigger_width_lines.create_button(
            self.sim.screen, (107, 142, 35), 50, 0, 50, 40, 0, "+WIDTH", (255, 255, 255))

        self.button_less_battery.create_button(
            self.sim.screen, (107, 142, 35), 130, 0, 50, 40, 0, "-BATT", (255, 255, 255))
        self.button_more_battery.create_button(
            self.sim.screen, (107, 142, 35), 180, 0, 50, 40, 0, "+BATT", (255, 255, 255))

        self.button_less_size.create_button(
            self.sim.screen, (107, 142, 35), 280, 0, 50, 40, 0, "-HEX", (255, 255, 255))
        self.button_more_size.create_button(
            self.sim.screen, (107, 142, 35), 330, 0, 50, 40, 0, "+HEX", (255, 255, 255))

        self.button_down.create_button(
            self.sim.screen, (107, 142, 35), 410, 0, 50, 40, 0, "-R", (255, 255, 255))
        self.button_up.create_button(
            self.sim.screen, (107, 142, 35), 460, 0, 50, 40, 0, "+R", (255, 255, 255))

        self.button_load_saved_points.create_button(
            self.sim.screen, (107, 142, 35), 620, 560, 50, 20, 0, "LOAD", (255, 255, 255))
        self.button_save_points.create_button(
            self.sim.screen, (107, 142, 35), 620, 580, 50, 20, 0, "SAVE", (255, 255, 255))

        self.button_more_height.create_button(
            self.sim.screen, (107, 142, 35), 590, 0, 50, 40, 0, "+H", (255, 255, 255))
        self.button_less_height.create_button(
            self.sim.screen, (107, 142, 35), 540, 0, 50, 40, 0, "-H", (255, 255, 255))

        # Draw button labels
        pygame.draw.rect(self.sim.screen, simulation.Sim.black, (100, 0, 30, 40))
        label = simulation.Sim.monoFont20.render(str(self.sim.line_width), 10, simulation.Sim.white)
        self.sim.screen.blit(label, (102, 10))

        pygame.draw.rect(self.sim.screen, simulation.Sim.black, (230, 0, 50, 40))
        label = simulation.Sim.monoFont20.render(str(self.sim.battery_autonomy), 10, simulation.Sim.white)
        self.sim.screen.blit(label, (230, 10))

        pygame.draw.rect(self.sim.screen, simulation.Sim.black, (380, 0, 30, 40))
        label = simulation.Sim.monoFont20.render(str(self.sim.radius), 10, simulation.Sim.white)
        self.sim.screen.blit(label, (382, 10))

        pygame.draw.rect(self.sim.screen, simulation.Sim.black, (510, 0, 30, 40))
        label = simulation.Sim.monoFont20.render(str(self.sim.number_of_robots), 10, simulation.Sim.white)
        self.sim.screen.blit(label, (512, 10))

        pygame.draw.rect(self.sim.screen, simulation.Sim.black, (640, 0, 30, 40))
        label = simulation.Sim.monoFont20.render(str(self.sim.robot_base_height), 10, simulation.Sim.white)
        self.sim.screen.blit(label, (642, 10))

        # pygame.draw.rect(self.sim.screen, simulation.Sim.black, (670, 0, 110, 40))
        # label = simulation.Sim.monoFont20.render(
        #     str(self.sim.user_selected_algo_description[self.sim.user_selected_algo]), 10, simulation.Sim.white)
        # self.sim.screen.blit(label, (675, 10))

        pygame.draw.rect(self.sim.screen, simulation.Sim.black, (670, 560, 80, 40))
        if self.sim.is_remove_mode_active:
            label = simulation.Sim.monoFont20.render("Remove", 10, simulation.Sim.white)
        else:
            label = simulation.Sim.monoFont20.render("Normal", 10, simulation.Sim.white)
        self.sim.screen.blit(label, (675, 570))

        pygame.draw.rect(self.sim.screen, simulation.Sim.black, (690, 0, 110, 40))
        label = simulation.Sim.monoFont20.render(
            str(self.sim.user_selected_algo_description[self.sim.user_selected_algo]), 10, simulation.Sim.white)
        self.sim.screen.blit(label, (695, 10))
        pass

    def handle_click(self, click_button, pos):

        sim = self.sim

        if self.button_up.pressed(pos):
            if sim.number_of_robots < 10:
                sim.number_of_robots += 1
                return

        if self.button_down.pressed(pos):
            if sim.number_of_robots > 1:
                sim.number_of_robots -= 1
                return

        if self.button_more_size.pressed(pos):
            if sim.radius <= 205:
                sim.radius += 5
                sim.hexes_wide = int(config.IMAGE_WIDTH / sim.radius)
                sim.hexes_high = int(config.IMAGE_HEIGHT / sim.radius) + 1

                sim.half_radius = sim.radius / 2.0
                sim.half_hex_height = sqrt(sim.radius ** 2 - sim.half_radius ** 2)

                sim.hex_list = math_helper.get_hex_point_list(sim.hexes_wide, sim.hexes_high,
                                                       sim.radius, sim.half_hex_height, config.THETA)

                sim.hex_intersection = []
                sim.robot_paths = {}
                sim.robot_path_angles = {}
                sim.obstacles_hexagon = set([])
            return

        if self.button_less_size.pressed(pos):
            if sim.radius > 6:
                sim.radius -= 5
                sim.hexes_wide = int(config.IMAGE_WIDTH / sim.radius)
                sim.hexes_high = int(config.IMAGE_HEIGHT / sim.radius) + 1

                sim.half_radius = sim.radius / 2.0
                sim.half_hex_height = sqrt(sim.radius ** 2 - sim.half_radius ** 2)

                sim.hex_list = math_helper.get_hex_point_list(sim.hexes_wide, sim.hexes_high,
                                                       sim.radius, sim.half_hex_height, config.THETA)

                sim.hex_intersection = []
                sim.robot_paths = {}
                sim.robot_path_angles = {}
                sim.obstacles_hexagon = set([])
            return

        if self.button_more_battery.pressed(pos):
            sim.battery_autonomy += 50
            sim.robot_waypoints['battery_autonomy'] = sim.battery_autonomy
            return

        if self.button_less_battery.pressed(pos):
            sim.battery_autonomy -= 50
            sim.robot_waypoints['battery_autonomy'] = sim.battery_autonomy
            return

        if self.button_bigger_width_lines.pressed(pos):
            sim.line_width += 1
            return

        if self.button_less_width_lines.pressed(pos):
            sim.line_width -= 1
            return

        if self.button_more_height.pressed(pos):
            sim.robot_base_height += 10
            return

        if self.button_less_height.pressed(pos):
            sim.robot_base_height -= 10
            return

        if self.button_load_saved_points.pressed(pos):
            if os.path.isfile(config.USER_PREFERENCE_FILE):
                with open(config.USER_PREFERENCE_FILE, 'r') as f:
                    data = json.load(f)
                    sim.hex_list = math_helper.get_hex_point_list(
                        sim.hexes_wide, sim.hexes_high, sim.radius, sim.half_hex_height, config.THETA)
                    sim.hex_intersection = []
                    sim.robot_paths = {}
                    sim.robot_path_angles = {}
                    sim.obstacles_hexagon = set([])
                    sim.point_list = data["point_list"]
                    sim.trajectory_start_pt = data["trajectory_start_pt"]
                pass
            return

        if self.button_save_points.pressed(pos):
            if len(sim.point_list) <= 0 or sim.trajectory_start_pt == (0,0):
                print "Cannot save point list empty or start point not defined"
                return

            data = {
               'point_list' : sim.point_list,
               'trajectory_start_pt' : sim.trajectory_start_pt
            }

            with open(config.USER_PREFERENCE_FILE, 'w+') as f:
                json.dump(data, f)

            print "Data saved to :", config.USER_PREFERENCE_FILE
            return

        if click_button[0]:
            sim.trajectory_start_pt = pos
            sim.robot_waypoints['start_point'] = pos
            pass

        if click_button[2]:
            if sim.is_remove_mode_active:
                p = Point((pos[0], pos[1]))
                for i in xrange(len(sim.hex_list)):
                    hex_poly = sim.hex_list[i]
                    if p.within(hex_poly) or p.intersects(hex_poly):
                        if i in sim.obstacles_hexagon:
                            sim.obstacles_hexagon.remove(i)
                        else:
                            sim.obstacles_hexagon.add(i)
                pass
                print "obstacles_hexagon", sim.obstacles_hexagon
            else:
                found = False
                for border in sim.point_list:
                    d, theta = math_helper.points_to_vector((pos[0], pos[1]), border)
                    if d < 10:
                        found = True
                        sim.point_list.append(border)
                        break

                if not found:
                    sim.point_list.append((pos[0], pos[1]))

            print "Left Click", pos[0], pos[1]
        pass

    pass
