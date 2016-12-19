#!/usr/bin/python

"""
gen_island_overworld.py - Script to test basic algorithms for tilemap-focused procedural generation methods
by mit-mit

Ideas and methods based on "Polygon Map Generation for Games" by Amit Patel
http://www-cs-students.stanford.edu/~amitp/game-programming/polygon-map-generation/

Uses modified version of 8x8 pixel "micro tileset - Overworld and Dungeon" by thKaspar
http://opengameart.org/content/micro-tileset-overworld-and-dungeon

Uses "pyshull" Delaunay triangulation implementation by Tim Sheerman-Chase
https://github.com/TimSC/pyshull
"""

import os
import numpy as np
from math import *
import random

from pyshull import pyshull
import Queue
import heapq

import pygame
from pygame.locals import *

# RenderTileMap - Class to render tiles
class RenderTileMap(object):
    def __init__(self):
        
        self.tiles = pygame.image.load('tilesetOnD_1x.png').convert()
        self.tiles.set_colorkey((255,0,255))
        
        self.tilesize = 8
        
        self.tiles_coords = []
        for j in xrange(32):
            for i in xrange(32):
                self.tiles_coords.append((i*self.tilesize, j*self.tilesize, self.tilesize, self.tilesize))
        
        # define tiles
        self.tileind = {}
        
        self.tileind['water'] = {}
        self.tileind['water']['flat'] = 448
        self.tileind['water']['text'] = [449,450,451]
        
        self.tileind['ice'] = {}
        self.tileind['ice']['flat'] = 20+384
        self.tileind['ice']['text'] = [1,2,3]
        self.tileind['ice']['coast'] = [9+384+12,10+384+12,11+384+12,9+416+12,10+416+12,11+416+12,9+448+12,10+448+12,11+448+12, \
            12+384+12,13+384+12,12+416+12,13+416+12]
        
        self.tileind['grass'] = {}
        self.tileind['grass']['flat'] = 0
        self.tileind['grass']['text'] = [1,2,3]
        self.tileind['grass']['dark'] = 32
        self.tileind['grass']['corner'] = [4,5,6,4+32,5+32,6+32,4+64,5+64,6+64]
        self.tileind['grass']['coast'] = [9,10,11,9+32,10+32,11+32,9+64,10+64,11+64, 12,13,12+32,13+32]
        
        self.tileind['darkgrass'] = {}
        self.tileind['darkgrass']['flat'] = 192
        self.tileind['darkgrass']['text'] = [1+192,2+192,3+192]
        self.tileind['darkgrass']['dark'] = 32+192
        self.tileind['darkgrass']['corner'] = [4+192,5+192,6+192,4+224,5+224,6+224,4+256,5+256,6+256]
        self.tileind['darkgrass']['coast'] = [9+192,10+192,11+192,9+224,10+224,11+224,9+256,10+256,11+256, 12+192,13+192,12+224,13+224]
        
        self.tileind['dirt'] = {}
        self.tileind['dirt']['flat'] = 96
        self.tileind['dirt']['text'] = [1+96,2+96,3+96]
        self.tileind['dirt']['dark'] = 32+96
        self.tileind['dirt']['corner'] = [4+96,5+96,6+96,4+128,5+128,6+128,4+160,5+160,6+160]
        self.tileind['dirt']['coast'] = [9+96,10+96,11+96,9+128,10+128,11+128,9+160,10+160,11+160, 12+96,13+96,12+128,13+128]
        
        self.tileind['darkdirt'] = {}
        self.tileind['darkdirt']['flat'] = 288
        self.tileind['darkdirt']['text'] = [1+288,2+288,3+288]
        self.tileind['darkdirt']['dark'] = 32+288
        self.tileind['darkdirt']['corner'] = [4+288,5+288,6+288,4+320,5+320,6+320,4+352,5+352,6+352]
        self.tileind['darkdirt']['coast'] = [9+288,10+288,11+288,9+320,10+320,11+320,9+352,10+352,11+352, 12+288,13+288,12+320,13+320]
        
        self.tileind['stone'] = {}
        self.tileind['stone']['flat'] = 480
        self.tileind['stone']['text'] = [1+480,2+480,3+480]
        self.tileind['stone']['dark'] = 512
        self.tileind['stone']['corner'] = [4+480,5+480,6+480,4+512,5+512,6+512,4+544,5+544,6+544]
        self.tileind['stone']['coast'] = [9+480,10+480,11+480,9+512,10+512,11+512,9+544,10+544,11+544, 12+480,13+480,12+512,13+512]
        
        self.tileind['snow'] = {}
        self.tileind['snow']['flat'] = 384
        self.tileind['snow']['text'] = [1+384,2+384,3+384]
        self.tileind['snow']['dark'] = 416
        self.tileind['snow']['corner'] = [4+384,5+384,6+384,4+416,5+416,6+416,4+448,5+448,6+448]
        self.tileind['snow']['coast'] = [9+384,10+384,11+384,9+416,10+416,11+416,9+448,10+448,11+448, 12+384,13+384,12+416,13+416]
        
        self.tileind['darksnow'] = {}
        self.tileind['darksnow']['flat'] = 576
        self.tileind['darksnow']['text'] = [1+576,2+576,3+576]
        self.tileind['darksnow']['dark'] = 512
        self.tileind['darksnow']['corner'] = [4+576,5+576,6+576,4+608,5+608,6+608,4+640,5+640,6+640]
        self.tileind['darksnow']['coast'] = [9+576,10+576,11+576,9+608,10+608,11+608,9+640,10+640,11+640, 12+576,13+576,12+608,13+608]
        
        # sand
        # glacier
        
        self.tileind['house'] = {}
        self.tileind['house']['tiles'] = [26,26+32,26+64,26+96]
        
        self.tileind['housesnow'] = {}
        self.tileind['housesnow']['tiles'] = [27,27+32,27+64,27+96]
        
        self.tileind['tree'] = {}
        self.tileind['tree']['flat'] = 14
        self.tileind['tree']['corner'] = [15,16,17, 15+32,16+32,17+32, 15+64,16+64,17+64, 18,19, 18+32,19+32]
        
        self.tileind['treesnow'] = {}
        self.tileind['treesnow']['flat'] = 14+384
        self.tileind['treesnow']['corner'] = [15+384,16+384,17+384, 15+416,16+416,17+416, 15+448,16+448,17+448, 18+384,19+384, 18+416,19+416]
        
        self.tileind['treepartialsnow'] = {}
        self.tileind['treepartialsnow']['flat'] = 20
        self.tileind['treepartialsnow']['corner'] = [21,22,23, 21+32,22+32,23+32, 21+64,22+64,23+64, 24,25, 24+32,25+32]
        
        self.tileind['mountain'] = {}
        self.tileind['mountain']['flat'] = 14+96
        self.tileind['mountain']['corner'] = [15+96,16+96,17+96, 15+128,16+128,17+128, 15+160,16+160,17+160, 18+96,19+96, 18+128,19+128]
        
        self.tileind['mountainsnow'] = {}
        self.tileind['mountainsnow']['flat'] = 20+96
        self.tileind['mountainsnow']['corner'] = [21+96,22+96,23+96, 21+128,22+128,23+128, 21+160,22+160,23+160, 24+96,25+96, 24+128,25+128]
    
    def DoTransition(self, map_data, map_size, from_cover, to_cover):
        for i in xrange(len(map_data)):
            if map_data[i] == to_cover:
                ix = i%map_size[0]
                iy = i/map_size[0]
                tileind = -1
                if ix == 0 or iy == 0 or ix == (map_size[0]-1) or iy == (map_size[1]-1):
                    continue
                if map_data[i+map_size[0]] == from_cover: # land south
                    if map_data[i+1] == from_cover:
                        tileind = self.tileind[map_data[i+map_size[0]]]['corner'][8]
                    elif map_data[i-1] == from_cover:
                        tileind = self.tileind[map_data[i+map_size[0]]]['corner'][6]
                    else:
                        tileind = self.tileind[map_data[i+map_size[0]]]['corner'][7]
                elif map_data[i-map_size[0]] == from_cover: # land north
                    if map_data[i+1] == from_cover:
                        tileind = self.tileind[map_data[i-map_size[0]]]['corner'][2]
                    elif map_data[i-1] == from_cover:
                        tileind = self.tileind[map_data[i-map_size[0]]]['corner'][0]
                    else:
                        tileind = self.tileind[map_data[i-map_size[0]]]['corner'][1]
                elif map_data[i-1] == from_cover: # land west
                    tileind = self.tileind[map_data[i-1]]['corner'][3]
                elif map_data[i+1] == from_cover: # land east
                    tileind = self.tileind[map_data[i+1]]['corner'][5]
                if tileind >= 0:
                    self.map_image.blit(self.tiles, (ix*self.tilesize, iy*self.tilesize), area=self.tiles_coords[tileind])
    
    def RenderOverworldMap(self, map_data, map_data_cover, map_data_text, map_size):
        
        self.map_image = pygame.Surface((self.tilesize*map_size[0],self.tilesize*map_size[1]))
        self.map_image.fill((0,0,0))
        self.map_image.convert()
        
        # first render plain ground
        for i in xrange(len(map_data)):
            ix = i%map_size[0]
            iy = i/map_size[0]
            tileind = self.tileind[map_data[i]]['flat']
            self.map_image.blit(self.tiles, (ix*self.tilesize, iy*self.tilesize), area=self.tiles_coords[tileind])
        
        # render textures (speckles)
        for i in xrange(len(map_data)):
            if map_data_text[i] == 'none':
                continue
            ix = i%map_size[0]
            iy = i/map_size[0]
            tileind = self.tileind[map_data[i]]['text'][map_data_text[i]]
            self.map_image.blit(self.tiles, (ix*self.tilesize, iy*self.tilesize), area=self.tiles_coords[tileind])
        
        # render coasts
        for i in xrange(len(map_data)):
            if map_data[i] == 'water':
                ix = i%map_size[0]
                iy = i/map_size[0]
                tileind = -1
                if ix == 0 or iy == 0 or ix == (map_size[0]-1) or iy == (map_size[1]-1):
                    continue
                if not map_data[i+map_size[0]] == 'water': # land south
                    if not map_data[i+1] == 'water':
                        tileind = self.tileind[map_data[i+map_size[0]]]['coast'][8]
                    elif not map_data[i-1] == 'water':
                        tileind = self.tileind[map_data[i+map_size[0]]]['coast'][6]
                    else:
                        tileind = self.tileind[map_data[i+map_size[0]]]['coast'][7]
                elif not map_data[i-map_size[0]] == 'water': # land north
                    if not map_data[i+1] == 'water':
                        tileind = self.tileind[map_data[i-map_size[0]]]['coast'][2]
                    elif not map_data[i-1] == 'water':
                        tileind = self.tileind[map_data[i-map_size[0]]]['coast'][0]
                    else:
                        tileind = self.tileind[map_data[i-map_size[0]]]['coast'][1]
                elif not map_data[i-1] == 'water': # land west
                    tileind = self.tileind[map_data[i-1]]['coast'][3]
                elif not map_data[i+1] == 'water': # land east
                    tileind = self.tileind[map_data[i+1]]['coast'][5]
                elif not map_data[i-1-map_size[0]] == 'water': # land NW
                    tileind = self.tileind[map_data[i-1-map_size[0]]]['coast'][12]
                elif not map_data[i+1-map_size[0]] == 'water': # land NE
                    tileind = self.tileind[map_data[i+1-map_size[0]]]['coast'][11]
                elif not map_data[i+1+map_size[0]] == 'water': # land SE
                    tileind = self.tileind[map_data[i+1+map_size[0]]]['coast'][9]
                elif not map_data[i-1+map_size[0]] == 'water': # land SW
                    tileind = self.tileind[map_data[i-1+map_size[0]]]['coast'][10]
                if tileind >= 0:
                    self.map_image.blit(self.tiles, (ix*self.tilesize, iy*self.tilesize), area=self.tiles_coords[tileind])
        
        
        # do transitions
        self.DoTransition(map_data, map_size, 'grass', 'darkgrass')
        self.DoTransition(map_data, map_size, 'snow', 'darksnow')
        self.DoTransition(map_data, map_size, 'darkgrass', 'darkdirt')
        self.DoTransition(map_data, map_size, 'darkgrass', 'dirt')
        self.DoTransition(map_data, map_size, 'snow', 'dirt')
        self.DoTransition(map_data, map_size, 'darksnow', 'darkdirt')
        self.DoTransition(map_data, map_size, 'darksnow', 'dirt')
        self.DoTransition(map_data, map_size, 'dirt', 'darkdirt')
        self.DoTransition(map_data, map_size, 'dirt', 'stone')
        self.DoTransition(map_data, map_size, 'darkdirt', 'stone')
        self.DoTransition(map_data, map_size, 'darkgrass', 'stone')
        self.DoTransition(map_data, map_size, 'darksnow', 'stone')
        self.DoTransition(map_data, map_size, 'grass', 'stone')
        self.DoTransition(map_data, map_size, 'snow', 'stone')
        
        self.DoTransition(map_data, map_size, 'snow', 'darkgrass')
        self.DoTransition(map_data, map_size, 'snow', 'grass')
        self.DoTransition(map_data, map_size, 'snow', 'ice')
        
        # now render trees/mountains
        for i in xrange(len(map_data_cover)):
            if map_data_cover[i] == 'none':
                continue
            ix = i%map_size[0]
            iy = i/map_size[0]
            if not 'flat' in self.tileind[map_data_cover[i]].keys():
                tileind = self.tileind[map_data_cover[i]]['tiles'][0]
                self.map_image.blit(self.tiles, (ix*self.tilesize, iy*self.tilesize), area=self.tiles_coords[tileind])
                continue
            surround_tiles = [map_data_cover[i-1+map_size[0]],map_data_cover[i+map_size[0]],map_data_cover[i+1+map_size[0]], \
                map_data_cover[i-1],map_data_cover[i+1],map_data_cover[i-1-map_size[0]],map_data_cover[i-map_size[0]],map_data_cover[i+1-map_size[0]]]
            other_around = sum([not t == map_data_cover[i] for t in surround_tiles])
            if other_around >= 5:
                tileind = self.tileind[map_data_cover[i]]['flat']
            elif not map_data_cover[i+map_size[0]] == map_data_cover[i]: # land south
                if not map_data_cover[i+1] == map_data_cover[i]:
                    tileind = self.tileind[map_data_cover[i]]['corner'][8]
                elif not map_data_cover[i-1] == map_data_cover[i]:
                    tileind = self.tileind[map_data_cover[i]]['corner'][6]
                else:
                    tileind = self.tileind[map_data_cover[i]]['corner'][7]
            elif not map_data_cover[i-map_size[0]] == map_data_cover[i]: # land north
                if not map_data_cover[i+1] == map_data_cover[i]:
                    tileind = self.tileind[map_data_cover[i]]['corner'][2]
                elif not map_data_cover[i-1] == map_data_cover[i]:
                    tileind = self.tileind[map_data_cover[i]]['corner'][0]
                else:
                    tileind = self.tileind[map_data_cover[i]]['corner'][1]
            elif not map_data_cover[i-1] == map_data_cover[i]: # land west
                tileind = self.tileind[map_data_cover[i]]['corner'][3]
            elif not map_data_cover[i+1] == map_data_cover[i]: # land east
                tileind = self.tileind[map_data_cover[i]]['corner'][5]
            elif not map_data_cover[i-1-map_size[0]] == map_data_cover[i]: # land NW
                tileind = self.tileind[map_data_cover[i]]['corner'][9]
            elif not map_data_cover[i+1-map_size[0]] == map_data_cover[i]: # land NE
                tileind = self.tileind[map_data_cover[i]]['corner'][10]
            elif not map_data_cover[i+1+map_size[0]] == map_data_cover[i]: # land SE
                tileind = self.tileind[map_data_cover[i]]['corner'][12]
            elif not map_data_cover[i-1+map_size[0]] == map_data_cover[i]: # land SW
                tileind = self.tileind[map_data_cover[i]]['corner'][11]
            else:
                tileind = self.tileind[map_data_cover[i]]['corner'][4]
            
            self.map_image.blit(self.tiles, (ix*self.tilesize, iy*self.tilesize), area=self.tiles_coords[tileind])
        
        # render villages and cave/dungeon entrances
        
        
    def SaveMap(self, path):
        pygame.image.save(self.map_image, path)

# CreateTileMap - Class to build procedurally-generated fantasy island maps
class OverworldMap(object):
    
    def __init__(self, Options=None):
        if Options == None: # define standard options
            self.Options = {}

            self.Options['map_size'] = [128,128]
            
            self.Options['Np'] = 1600 # number of random cels to begin with
            self.Options['grid_div'] = 40 # number of grid divisions to perform for making stratified random points (reduce clumpiness)
            self.Options['Nlandstart'] = 6 # number of randomdised land seed points
            
            self.Options['num_lakes'] = 3 # number of lakes to generate
            self.Options['lake_size'] = 3 # flood fill distance of lakes (in points)
            self.Options['lake_min_coast_dist'] = 4 # minimum distance to the coast
            self.Options['lake_max_coast_dist'] = 7 # maximum distance to the coast
        else:
            self.Options = Options
    
    def CreateVillage(self, loc, type='coastal', size='small', season='regular'):
        ms = self.Options['map_size']
        if size == 'small':
            inds = [loc,loc+1,loc+ms[0],loc+1+ms[0]]
            inds_border = [loc-1-ms[0],loc-ms[0],loc+1-ms[0],loc+2-ms[0],loc-1,loc+2,loc-1+ms[0],loc+2+ms[0], \
                loc-1+2*ms[0],loc+2*ms[0],loc+1+2*ms[0],loc+2+2*ms[0]]
        elif size == 'large':
            inds = [loc-1-ms[0],loc-ms[0],loc+1-ms[0],loc+2-ms[0],loc-1,loc,loc+1,loc+2,loc-1+ms[0],loc+ms[0],loc+1+ms[0],loc+2+ms[0]]
            inds_border = [loc-2-2*ms[0],loc-1-2*ms[0],loc-2*ms[0],loc+1-2*ms[0],loc+2-2*ms[0],loc+3-2*ms[0], \
                loc-2-ms[0],loc-2,loc-2+ms[0],loc+3-ms[0],loc+3,loc+3+ms[0], \
                loc-2+2*ms[0],loc-1+2*ms[0],loc+2*ms[0],loc+1+2*ms[0],loc+2+2*ms[0],loc+3+2*ms[0]]
        if type == 'coastal':
            ground_border = 'grass'
            ground_under = 'darkgrass'
        elif type == 'forest':
            ground_border = 'darkgrass'
            ground_under = 'grass'
        elif type == 'mountain':
            ground_border = 'dirt'
            ground_under = 'stone'
        
        if season == 'winter' and not type == 'mountain':
            ground_border = 'snow'
            ground_under = 'dirt'
        
        # clear/draw surroundings
        for i in inds_border:
            self.map_data_cover[i] = 'none'
            self.map_data_text[i] = 'none'
            if self.map_data[i] == 'stone':
                pass
            else:
                self.map_data[i] = ground_border
        for i in inds:
            self.map_data_cover[i] = 'none'
            self.map_data_text[i] = 'none'
            self.map_data[i] = ground_under
        
        # draw houses
        if season == 'summer':
            house_type = 'house'
        elif season == 'winter':
            house_type = 'housesnow'
        if size == 'small':
            self.map_data_cover[inds[0]] = house_type
            self.map_data_cover[inds[3]] = house_type
        elif size == 'large':
            self.map_data_cover[inds[0]] = house_type
            self.map_data_cover[inds[1]] = house_type
            self.map_data_cover[inds[4]] = house_type
            self.map_data_cover[inds[7]] = house_type
            self.map_data_cover[inds[10]] = house_type
    
    def draw_support(self,cent,size,type):
        ms = self.Options['map_size']
        if size == 1:
            ps = [cent]
        elif size == 3:
            ps = [cent-1-ms[0],cent-ms[0],cent+1-ms[0], \
                cent-1,cent,cent+1, \
                cent-1+ms[0],cent+ms[0],cent+1+ms[0]]
        if not type == 'none':
            self.map_data[ps[4]] = type
        for p in ps:
            self.map_data_cover[p] = 'none'
    
    def RasterisePath(self, path, points, type):
        
        #draw_locs = []
        
        ms = self.Options['map_size']
        for i in xrange(len(path)-1):
            p1 = path[i]
            x1 = points[p1][0]
            y1 = points[p1][1]
            p2 = path[i+1]
            x2 = points[p2][0]
            y2 = points[p2][1]
            if x1 == x2:
                ix = x1
                for iy in xrange(min(y1,y2),max(y1,y2)+1):
                    self.draw_support(iy*ms[0]+ix,3,type)
                    #draw_locs.append((ix,iy))
            else:
                deltax = float(x2 - x1)
                if deltax > 0:
                    rsign = 1
                else:
                    rsign = -1
                deltay = float(y2 - y1)
                if deltay == 0:
                    ysign = 0
                elif deltay > 0:
                    ysign = 1
                else:
                    ysign = -1
                error = -1.0
                deltaerr = fabs(deltay / deltax)
                iy = y1
                for ix in xrange(x1,x2+rsign,rsign): 
                    self.draw_support(iy*ms[0]+ix,3,type)
                    #draw_locs.append((ix,iy))
                    if ix == x2:
                        continue
                    error += deltaerr
                    while error >= 0.0:
                        iy += ysign
                        self.draw_support(iy*ms[0]+ix,3,type)
                        #draw_locs.append((ix,iy))
                        error -= 1.0
    
    def ClearPath2Sea(self, point_ind, points, neighbours, distance2sea, distance2freshwater):
        
        # depth first search to coast
        came_from = [-1 for i in xrange(len(points))]
        came_from[point_ind] = 0
        reached_sea = False
        current_point = point_ind
        while not reached_sea:
            print "current point: %d"%(current_point)
            cost = []
            for ni in neighbours[current_point]:
                if distance2freshwater[ni] > 1 and came_from[ni] == -1:
                    cost.append(distance2sea[ni])
                else:
                    cost.append(100)
            print "neighbours: ", neighbours[current_point]
            print "costs: ", cost
            min_cost = min(cost)
            if min_cost == 100:
                current_point = came_from[current_point]
                print "moved back to %d"%(current_point)
                if current_point == 0:
                    print "failed to find a way"
                    break
            else:
                next_point = neighbours[current_point][cost.index(min_cost)]
                print "moving to point: %d, distance: %f"%(next_point,min_cost)
                came_from[next_point] = current_point
                current_point = next_point
                if distance2sea[current_point] <= 2.0:
                    reached_sea = True
                    print "hit sea"
        
        if reached_sea:
            path = [current_point]
            while not current_point == point_ind:
                current_point = came_from[current_point]
                path.append(current_point)
            self.RasterisePath(path, points, 'none')
    
    def ClearPathBetweenPoints(self, point_ind1, point_ind2, points, neighbours, distance2sea, distance2freshwater):
        
        point_dists = []
        for i in xrange(len(points)):
            #dist = sqrt(pow(points[i][0]-points[point_ind2][0],2)+pow(points[i][1]-points[point_ind2][1],2))
            dist = abs(points[i][0]-points[point_ind2][0]) + abs(points[i][1]-points[point_ind2][1])
            point_dists.append(dist)
        
        frontier = []
        heapq.heappush(frontier, (0, point_ind1))
        came_from = [-1 for i in xrange(len(points))]
        came_from[point_ind1] = None
        cost_so_far = [0 for i in xrange(len(points))]
        cost_so_far[point_ind1] = 0
        
        while True:
            if len(frontier) == 0: # ran out of options, no path
                print "no path found"
                return []
            current = heapq.heappop(frontier)[1]
            
            if current == point_ind2:  # got it
                path = [current]
                while not came_from[current] == None: # step backwards through came froms to get path
                    current = came_from[current]
                    path.insert(0,current)
                print path
                self.RasterisePath(path, points, 'stone')
                return
                #return (path, draw_locs)
            
            for ni in neighbours[current]:
                if distance2sea[ni] <= 1 or distance2freshwater[ni] <= 1:
                    next = (ni,10000)
                else:
                    next = (ni,1)
                new_cost = cost_so_far[current] + next[1]
                if came_from[next[0]] == -1 or new_cost < cost_so_far[next[0]]:
                    cost_so_far[next[0]] = new_cost
                    priority = new_cost + point_dists[ni]
                    heapq.heappush(frontier, (priority, next[0]))
                    came_from[next[0]] = current
    
    def BuildOverworldMap(self, season='summer', verbose=False, seed=None):
        
        if not seed == None:
            random.seed(seed)
            np.random.seed(seed)
        
        # Initialise data
        self.map_data = ['water' for i in xrange(self.Options['map_size'][0]*self.Options['map_size'][1])]
        self.map_data_cover = ['none' for i in xrange(self.Options['map_size'][0]*self.Options['map_size'][1])]
        self.map_data_text = ['none' for i in xrange(self.Options['map_size'][0]*self.Options['map_size'][1])]
        
        # Generate initial random point set
        if verbose == True:
            print "generating points ..."

        points = []
        Ni = int(self.Options['Np']/pow(self.Options['grid_div'],2))
        xinc = (self.Options['map_size'][0]/self.Options['grid_div'])
        yinc = (self.Options['map_size'][1]/self.Options['grid_div'])
        for xi in xrange(self.Options['grid_div']):
            for yi in xrange(self.Options['grid_div']):
                start_x = xi*xinc
                start_y = yi*yinc
                points.extend([(np.random.randint(start_x,start_x+xinc),np.random.randint(start_y,start_y+yinc)) for i in xrange(Ni)])
        points = list(set(points))
        
        # remove existing points on boundaries and add own sea points
        pr = []
        for i in xrange(len(points)):
            if points[i][0] == 0 or points[i][1] == 0 or points[i][0] == (self.Options['map_size'][0]-1) or points[i][1] == (self.Options['map_size'][1]-1):
                pr.append(points[i])
        for p in pr:
            points.remove(p)
        
        Nwalls = int(sqrt(self.Options['Np']))
        for i in xrange(Nwalls):
            step = i*self.Options['map_size'][0]/Nwalls
            points.append((0,step))
            points.append((self.Options['map_size'][0]-1,step))
            if i > 0:
                points.append((step,0))
            points.append((step,self.Options['map_size'][1]-1))
        
        # Compute delaunay trianglation of points to setup initial spatial networks
        if verbose == True:
            print "performing triangulation ..."
        
        triangles = pyshull.PySHull(np.array(points))
        neighbours = [[] for i in xrange(len(points))]
        for t in triangles:
            neighbours[t[0]].extend([t[1],t[2]])
            neighbours[t[1]].extend([t[0],t[2]])
            neighbours[t[2]].extend([t[0],t[1]])
        
        #################################################
        # Use floodfills to built high-level coastlines
        
        if verbose == True:
            print "creating land ..."
        
        if verbose == True:
            print "coastal floodfills ..."
        
        point_types = [-1 for i in xrange(len(points))]
        
        # water long boundaries
        frontier_water = Queue.Queue()
        for i in xrange(len(points)-4*Nwalls,len(points)): 
            point_types[i] = 0
            frontier_water.put(i)
        
        # random land starting points
        frontier_land = Queue.Queue()
        potential_points = [i for i in xrange(len(points)-4*Nwalls) if len(neighbours[i]) > 0]
        random.shuffle(potential_points)
        land_start = []
        i = 0
        boundary_size = 0.1
        for j in xrange(2):
            quadrats = [False,False,False,False]
            while not all(quadrats):
                ind = potential_points[i]
                if points[ind][0] < boundary_size*self.Options['map_size'][0] or points[ind][0] > (1.0-boundary_size)*self.Options['map_size'][0] or \
                    points[ind][1] < boundary_size*self.Options['map_size'][1] or points[ind][1] > (1.0-boundary_size)*self.Options['map_size'][1]:
                    i += 1
                    continue
                if points[ind][0] < self.Options['map_size'][0]/2:
                    if points[ind][1] < self.Options['map_size'][1]/2:
                        quad = 0
                    else:
                        quad = 2
                else:
                    if points[ind][1] < self.Options['map_size'][1]/2:
                        quad = 1
                    else:
                        quad = 3
                if quadrats[quad] == False:
                    quadrats[quad] = True
                    land_start.append(ind)
                i += 1
        
        for i in land_start:
            point_types[i] = 1
            frontier_land.put(i)
        
        # perform flood fills to allocate land/sea
        while not frontier_water.empty() and not frontier_land.empty():
            for i in xrange(frontier_land.qsize()): # empty queue of frontier objects only
                current_land = frontier_land.get()
                for next in neighbours[current_land]:
                    if point_types[next] == -1:
                        point_types[next] = 1
                        frontier_land.put(next)
            for i in xrange(frontier_water.qsize()): # empty queue of frontier objects only
                current_water = frontier_water.get()
                for next in neighbours[current_water]:
                    if point_types[next] == -1:
                        point_types[next] = 0
                        frontier_water.put(next)
        
        # Create distance to sea layer
        distance2sea = [0 for i in xrange(len(points))]
        touched = [-1 for i in xrange(len(points))]
        for i in xrange(len(points)):
            if len(neighbours[i]) == 0 or point_types[i] == 0:
                continue
            frontier = Queue.Queue()
            frontier.put((i,0))
            touched[i] = i
            found_sea = False
            while not found_sea:
                (current_point,dist) = frontier.get()
                for next_point in neighbours[current_point]:
                    if point_types[next_point] == 0:
                        found_sea = True
                        distance2sea[i] = dist+1
                        break
                    elif touched[next_point] < i:
                        touched[next_point] = i
                        frontier.put((next_point,dist+1))
        
        # Create distance to land layer (for sea points)
        distance2land = [0 for i in xrange(len(points))]
        touched = [-1 for i in xrange(len(points))]
        for i in xrange(len(points)):
            if len(neighbours[i]) == 0 or point_types[i] == 1:
                continue
            frontier = Queue.Queue()
            frontier.put((i,0))
            touched[i] = i
            found_land = False
            while not found_land:
                (current_point,dist) = frontier.get()
                for next_point in neighbours[current_point]:
                    if point_types[next_point] == 1:
                        found_land = True
                        distance2land[i] = dist+1
                        break
                    elif touched[next_point] < i:
                        touched[next_point] = i
                        frontier.put((next_point,dist+1))
        
        # Create some lakes
        
        if verbose == True:
            print "lakes ..."
        
        potential_lakes = [i for i in xrange(len(points)) if point_types[i] == 1 and \
            distance2sea[i] >= self.Options['lake_min_coast_dist'] and distance2sea[i] <= self.Options['lake_max_coast_dist']]
        random.shuffle(potential_lakes)
        touched = [-1 for i in xrange(len(points))]
        for i in xrange(self.Options['num_lakes']):
            lake_start = potential_lakes[i]
            frontier = Queue.Queue()
            frontier.put(lake_start)
            touched[lake_start] = i
            point_types[lake_start] = 2
            for j in xrange(self.Options['lake_size']):
                for k in xrange(frontier.qsize()): # empty queue of frontier objects only
                    current_lake = frontier.get()
                    for next in neighbours[current_lake]:
                        if point_types[next] == 1 and distance2sea[next] >= self.Options['lake_min_coast_dist'] and \
                            distance2sea[next] <= self.Options['lake_max_coast_dist'] and touched[next] < i:
                            point_types[next] = 2
                            touched[next] = i
                            frontier.put(next)
        
        # Create distance to freshwater layer
        distance2freshwater = [0 for i in xrange(len(points))]
        touched = [-1 for i in xrange(len(points))]
        for i in xrange(len(points)):
            if len(neighbours[i]) == 0 or not point_types[i] == 1:
                continue
            frontier = Queue.Queue()
            frontier.put((i,0))
            touched[i] = i
            found_water = False
            while not found_water and not frontier.empty():
                (current_point,dist) = frontier.get()
                for next_point in neighbours[current_point]:
                    if point_types[next_point] == 2:
                        found_water = True
                        distance2freshwater[i] = dist+1
                        break
                    elif touched[next_point] < i and point_types[next_point] == 1:
                        touched[next_point] = i
                        frontier.put((next_point,dist+1))
            if not found_water:
                distance2freshwater[i] = -1
        
        if verbose == True:
            print "rasterising ..."
        
        # Perform flood fills on full rasters
        def in_triangle(point, tv1, tv2, tv3):
            def sign(p1,p2,p3):
                return (p1[0] - p3[0])*(p2[1] - p3[1]) - (p2[0] - p3[0])*(p1[1] - p3[1]);
            if sign(point,tv1,tv2) <= 0.0 and sign(point,tv2,tv3) <= 0.0 and sign(point,tv3,tv1) <= 0.0:
                return True
            else:
                return False
        
        for t in triangles:
            
            xmin = min(points[t[0]][0],points[t[1]][0],points[t[2]][0])
            xmax = max(points[t[0]][0],points[t[1]][0],points[t[2]][0])
            ymin = min(points[t[0]][1],points[t[1]][1],points[t[2]][1])
            ymax = max(points[t[0]][1],points[t[1]][1],points[t[2]][1])
            
            if point_types[t[0]] > 0 and point_types[t[1]] > 0 and point_types[t[2]] > 0: # not sea
                
                if point_types[t[0]] == 2 and point_types[t[1]] == 2 and point_types[t[2]] == 2:
                    tri_type = 2
                else:
                    tri_type = 1
                
                dist2sea = (distance2sea[t[0]] + distance2sea[t[1]] + distance2sea[t[2]])/3.0
                dist2freshwater = (distance2freshwater[t[0]] + distance2freshwater[t[1]] + distance2freshwater[t[2]])/3.0
                
                tree_prob = 0
                mountain_prob = 0
                mountain_snow_prob = 0
                dirt_speck_prob = 0
                
                if tri_type == 2:
                    map_val = 'water'
                elif tri_type == 1:
                    if dist2sea <= 2:
                        if season == 'summer':
                            map_val = 'grass'
                        elif season == 'winter':
                            map_val = 'snow'
                    elif dist2sea <= 3:
                        if season == 'summer':
                            map_val = 'darkgrass'
                        elif season == 'winter':
                            map_val = 'darksnow'
                        tree_prob = 0.2
                    elif dist2sea <= 5:
                        if season == 'summer':
                            map_val = 'darkgrass'
                        elif season == 'winter':
                            map_val = 'darksnow'
                        tree_prob = 1.0
                    elif dist2sea <= 6:
                        if season == 'summer':
                            map_val = 'darkgrass'
                        elif season == 'winter':
                            map_val = 'darksnow'
                        tree_prob = 0.2
                    elif dist2sea <= 7:
                        map_val = 'darkdirt'
                    elif dist2sea <= 8:
                        map_val = 'dirt'
                    else:
                        map_val = 'dirt'
                        mountain_prob = 1.0
                        mountain_snow_prob = 0.9
                
                if dist2freshwater <= 1:
                    tree_prob = 0
                    mountain_prob = 0
                    mountain_snow_prob = 0
                
                for ix in xrange(xmin,xmax+1):
                    for iy in xrange(ymin,ymax+1):
                        if in_triangle([ix,iy], points[t[0]], points[t[1]], points[t[2]]):
                            self.map_data[iy*self.Options['map_size'][0]+ix] = map_val
                            if tree_prob > 0:
                                if random.uniform(0,1) < tree_prob:
                                    if season == 'summer':
                                        self.map_data_cover[iy*self.Options['map_size'][0]+ix] = 'tree'
                                    elif season == 'winter':
                                        self.map_data_cover[iy*self.Options['map_size'][0]+ix] = 'treesnow'
                            if mountain_prob > 0:
                                if random.uniform(0,1) < mountain_prob:
                                    if random.uniform(0,1) < mountain_snow_prob:
                                        self.map_data_cover[iy*self.Options['map_size'][0]+ix] = 'mountainsnow'
                                    else:
                                        self.map_data_cover[iy*self.Options['map_size'][0]+ix] = 'mountain'
        
        # decide on village and cave/dungeon entrance locations
        
        # coastal villages
        coastal_village_locs = []
        coastal_village_points = []
        
        potential_village = [i for i in xrange(len(points)) if point_types[i] == 1 and \
            distance2sea[i] >= 0.5 and distance2sea[i] <= 1.5]
        random.shuffle(potential_village)
        coastal_village_points.append(potential_village[0])
        ixs = [points[p][0] for p in potential_village]
        iys = [points[p][1] for p in potential_village]
        
        ix = ixs[0] # first village
        iy = iys[0]
        loc = iy*self.Options['map_size'][0]+ix
        coastal_village_locs.append(loc)
        
        dists = [sqrt(pow(ix-x,2)+pow(iy-y,2)) for (x,y) in zip(ixs,iys)] # second village, max dist away
        ind = dists.index(max(dists))
        coastal_village_points.append(potential_village[ind])
        ix2 = ixs[ind]
        iy2 = iys[ind]
        loc = iy2*self.Options['map_size'][0]+ix2
        coastal_village_locs.append(loc)
        
        # forest villages
        forest_village_locs = []
        forest_village_points = []
        potential_village = [i for i in xrange(len(points)) if point_types[i] == 1 and \
            distance2sea[i] >= 5 and distance2sea[i] <= 6]
        random.shuffle(potential_village)
        
        ixs = [points[p][0] for p in potential_village]
        iys = [points[p][1] for p in potential_village]
        
        dists = [sqrt(pow(ix-x,2)+pow(iy-y,2))+sqrt(pow(ix2-x,2)+pow(iy2-y,2)) for (x,y) in zip(ixs,iys)] # forest village, max dist away from coastal ones
        ind = dists.index(max(dists))
        forest_village_points.append(potential_village[ind])
        ix3 = ixs[ind]
        iy3 = iys[ind]
        loc = iy3*self.Options['map_size'][0]+ix3
        
        forest_village_locs.append(loc)
        
        # mountain village
        mountain_village_locs = []
        mountain_village_points = []
        potential_village = [i for i in xrange(len(points)) if point_types[i] == 1 and \
            distance2sea[i] >= (max(distance2sea)-1)]
        random.shuffle(potential_village)
        mountain_village_points.append(potential_village[0])
        loc = points[potential_village[0]][1]*self.Options['map_size'][0]+points[potential_village[0]][0]
        mountain_village_locs.append(loc)
        
        # build/clear roads/paths
        self.ClearPathBetweenPoints(coastal_village_points[0], mountain_village_points[0], points, neighbours, distance2sea, distance2freshwater)
        self.ClearPathBetweenPoints(forest_village_points[0], mountain_village_points[0], points, neighbours, distance2sea, distance2freshwater)
        
        self.ClearPath2Sea(forest_village_points[0], points, neighbours, distance2sea, distance2freshwater)
        
        # actually put villages/dungeon entrances down
        self.CreateVillage(coastal_village_locs[0], type='coastal', size='large', season=season)
        self.CreateVillage(coastal_village_locs[1], type='coastal', size='small', season=season)
        self.CreateVillage(forest_village_locs[0], type='forest', size='small', season=season)
        self.CreateVillage(mountain_village_locs[0], type='mountain', size='large', season=season)
        
        # Add in after effects
        water_spec_prob = 0.0
        grass_spec_prob = 0.05
        snow_spec_prob = 0.05
        darkgrass_spec_prob = 0.0
        dirt_spec_prob = 0.1
        darkdirt_spec_prob = 0.1
        
        for i in xrange(len(self.map_data)):
            if not self.map_data_cover[i] == 'none':
                continue
            ix = i%self.Options['map_size'][0]
            iy = i/self.Options['map_size'][0]
            if self.map_data[i] == 'water':
                if random.uniform(0,1) < water_spec_prob:
                    self.map_data_text[iy*self.Options['map_size'][0]+ix] = random.randint(0,2)
            elif self.map_data[i] == 'grass':
                if random.uniform(0,1) < grass_spec_prob:
                    self.map_data_text[iy*self.Options['map_size'][0]+ix] = random.randint(0,2)
            elif self.map_data[i] == 'snow':
                if random.uniform(0,1) < snow_spec_prob:
                    self.map_data_text[iy*self.Options['map_size'][0]+ix] = random.randint(0,2)
            elif self.map_data[i] == 'darkgrass':
                if random.uniform(0,1) < darkgrass_spec_prob:
                    self.map_data_text[iy*self.Options['map_size'][0]+ix] = random.randint(0,2)
            elif self.map_data[i] == 'dirt':
                if random.uniform(0,1) < dirt_spec_prob:
                    self.map_data_text[iy*self.Options['map_size'][0]+ix] = random.randint(0,2)
            elif self.map_data[i] == 'darkdirt':
                if random.uniform(0,1) < darkdirt_spec_prob:
                    self.map_data_text[iy*self.Options['map_size'][0]+ix] = random.randint(0,2)
        
        if season == 'winter': # Create a few icebergs
            potential_points = [i for i in xrange(len(points)-4*Nwalls) if len(neighbours[i]) > 0 and distance2land[i] >= 1]
            random.shuffle(potential_points)
            touched = [-1 for i in xrange(len(points))]
            for i in xrange(12):
                ice_start = potential_points[i]
                frontier = Queue.Queue()
                frontier.put(ice_start)
                touched[ice_start] = i
                point_types[ice_start] = 2
                for j in xrange(2):
                    for k in xrange(frontier.qsize()): # empty queue of frontier objects only
                        current_ice = frontier.get()
                        for next in neighbours[current_ice]:
                            if point_types[next] == 0 and distance2land[next] >= 1 and distance2land[next] <= 3 and touched[next] < i:
                                point_types[next] = 3
                                touched[next] = i
                                frontier.put(next)
            
            # raster them in
            for t in triangles:
                xmin = min(points[t[0]][0],points[t[1]][0],points[t[2]][0])
                xmax = max(points[t[0]][0],points[t[1]][0],points[t[2]][0])
                ymin = min(points[t[0]][1],points[t[1]][1],points[t[2]][1])
                ymax = max(points[t[0]][1],points[t[1]][1],points[t[2]][1])
                if point_types[t[0]] == 3 and point_types[t[1]] == 3 and point_types[t[2]] == 3: # iceberg
                    for ix in xrange(xmin,xmax+1):
                        for iy in xrange(ymin,ymax+1):
                            if in_triangle([ix,iy], points[t[0]], points[t[1]], points[t[2]]):
                                self.map_data[iy*self.Options['map_size'][0]+ix] = 'ice'
        
        # Fix-up raster to fill-in sea point surrounded by too much land
        changed = True
        while changed:
            changed = False
            for i in xrange(len(self.map_data)):
                if self.map_data[i] == 'water':
                    ix = i%self.Options['map_size'][0]
                    iy = i/self.Options['map_size'][0]
                    if ix == 0 or iy == 0 or ix == (self.Options['map_size'][0]-1) or iy == (self.Options['map_size'][1]-1):
                        continue
                    type_around = [self.map_data[i+1], self.map_data[i+1-self.Options['map_size'][0]], self.map_data[i+1+self.Options['map_size'][0]], \
                        self.map_data[i-1], self.map_data[i-1-self.Options['map_size'][0]], self.map_data[i-1+self.Options['map_size'][0]], \
                        self.map_data[i-self.Options['map_size'][0]], self.map_data[i+self.Options['map_size'][0]]]
                    land_around = sum([not t == 'water' for t in type_around])
                    if land_around >= 5:
                        changed = True
                        other_types = [t for t in type_around if not t == 'water' or t == 'stone']
                        if len(other_types) == 0:
                            self.map_data[i] = 'dirt'
                        else:
                            self.map_data[i] = max(set(other_types), key=other_types.count)
        
        # Fix-up raster to fill-in dark dirt points surrounded by too much land
        changed = True
        while changed:
            changed = False
            for i in xrange(len(self.map_data)):
                if self.map_data[i] == 'darkdirt':
                    ix = i%self.Options['map_size'][0]
                    iy = i/self.Options['map_size'][0]
                    if ix == 0 or iy == 0 or ix == (self.Options['map_size'][0]-1) or iy == (self.Options['map_size'][1]-1):
                        continue
                    type_around = [self.map_data[i+1], self.map_data[i+1-self.Options['map_size'][0]], self.map_data[i+1+self.Options['map_size'][0]], \
                        self.map_data[i-1], self.map_data[i-1-self.Options['map_size'][0]], self.map_data[i-1+self.Options['map_size'][0]], \
                        self.map_data[i-self.Options['map_size'][0]], self.map_data[i+self.Options['map_size'][0]]]
                    land_around = sum([not (t == 'darkdirt' or t == 'stone') for t in type_around])
                    if land_around >= 5:
                        changed = True
                        other_types = [t for t in type_around if not t == 'darkdirt']
                        self.map_data[i] = max(set(other_types), key=other_types.count)

####################################
# main script starts here

# Initialise pygame (for rendering)
pygame.init()
screen = pygame.display.set_mode((640,480))

# Create the overworld map
overworldmap = OverworldMap()
overworldmap.BuildOverworldMap(season='winter',seed=7,verbose=True)

# render the map, and save to png
rendertilemap = RenderTileMap()
rendertilemap.RenderOverworldMap(overworldmap.map_data, overworldmap.map_data_cover, overworldmap.map_data_text, overworldmap.Options['map_size'])
rendertilemap.SaveMap('map001.png')

# clean up
pygame.quit()
