#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <road_surface_recognition/map.h>

map_t *map_alloc(void)
{
    map_t *map;
    
    map = (map_t*) malloc(sizeof(map_t));

    map->origin_x = 0;
    map->origin_y = 0;

    map->size_x = 0;
    map->size_y = 0;
    map->scale = 0;
    
    map->cells = (map_cell_t*) NULL;

    return map;
}


void map_free(map_t *map)
{
    free(map->cells);
    free(map);
    return;
}


void map_update_cell(map_t *map, double gx, double gy, double data)
{
    int mi = MAP_GXWX(map, gx), mj = MAP_GYWY(map, gy);

    if (!MAP_VALID(map, mi, mj))
        return;

    map->cells[MAP_INDEX(map,mi,mj)].visit = map->cells[MAP_INDEX(map,mi,mj)].visit + 1;
    map->cells[MAP_INDEX(map,mi,mj)].average = 
        ((map->cells[MAP_INDEX(map,mi,mj)].average*(map->cells[MAP_INDEX(map,mi,mj)].visit-1))  + data)
        / map->cells[MAP_INDEX(map,mi,mj)].visit;

    //if(map->cells[MAP_INDEX(map,mi,mj)].average >= 50)
    //    map->cells[MAP_INDEX(map,mi,mj)].average = 100;
    //else
    //    map->cells[MAP_INDEX(map,mi,mj)].average = 0.1;

    if(map->cells[MAP_INDEX(map,mi,mj)].average != 100)
        map->cells[MAP_INDEX(map,mi,mj)].average = 0.1;
}

