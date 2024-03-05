/*
 * Copyright (C) C. De Wagter
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/computer_vision/cv_georeference.c"
 * @author C. De Wagter
 * Geo-reference computer vision detections
 */

#include "modules/computer_vision/cv_georeference.h"
#include "math/pprz_trig_int.h"
#include "math/pprz_algebra.h"
#include "math/pprz_algebra_int.h"

#include "state.h"
#include "generated/flight_plan.h"//here should be delete 
#include "modules/datalink/downlink.h"

#include "cv_detect_color_object.h"

//the root place should be ./sw/airborne
//add the reconstruc functions
#include "firmwares/rotorcraft/navigation.h"
#define debug 1

struct georeference_filter_t {
  struct Int32Vect3 x;          ///< Target
  struct Int32Vect3 v;          ///< Target Velocity
  int32_t P;                    ///< Covariance/Average-count
};

struct georeference_t {
  struct Int32Vect3 target_i;   ///< Target in pixels, with z being the focal length in pixels, x=up,y=right,out
  struct Int32Vect3 target_l;   ///< Target in meters, relative to the drone in LTP frame

  struct Int32Vect3 x_t;        ///< Target coordinates NED

  struct georeference_filter_t filter;  ///< Filter waypoint location
};

struct georeference_t geo;

void georeference_project(struct camera_frame_t *tar, int wp)
{
  // Target direction in camera frame: Zero is looking down in body frames
  // Pixel with x (width) value 0 projects to the left (body-Y-axis)
  // and y = 0 (height) to the top (body-X-axis)
  VECT3_ASSIGN(geo.target_i,
               ((tar->h / 2) - tar->py),
               (tar->px - (tar->w / 2)),
               (tar->f)
              );
  INT32_VECT3_LSHIFT(geo.target_i, geo.target_i, 4)

  // Camera <-> Body
  // Looking down in body frame
  // Bebop has 180deg Z rotation in camera (butt up yields normal webcam)
  struct Int32RMat body_to_cam_rmat;
  INT32_MAT33_ZERO(body_to_cam_rmat);
  MAT33_ELMT(body_to_cam_rmat, 0, 0) = -1 << INT32_TRIG_FRAC;
  MAT33_ELMT(body_to_cam_rmat, 1, 1) = -1 << INT32_TRIG_FRAC;
  MAT33_ELMT(body_to_cam_rmat, 2, 2) =  1 << INT32_TRIG_FRAC;

  struct Int32Vect3 target_b;
  int32_rmat_transp_vmult(&target_b, &body_to_cam_rmat, &geo.target_i);

  // Body <-> LTP
  struct Int32RMat *ltp_to_body_rmat = stateGetNedToBodyRMat_i();
  int32_rmat_transp_vmult(&geo.target_l, ltp_to_body_rmat, &target_b);

  // target_l is now a scale-less [pix<<POS_FRAC] vector in LTP from the drone to the target
  // Divide by z-component to normalize the projection vector
  int32_t zi = geo.target_l.z;
  if (zi <= 0)
  {
    // Pointing up or horizontal -> no ground projection
    return;
  }

  // Multiply with height above ground
  struct NedCoor_i *pos = stateGetPositionNed_i();
  int32_t zb = pos->z;
  geo.target_l.x *= zb;
  geo.target_l.y *= zb;

  // Divide by z-component
  geo.target_l.x /= zi;
  geo.target_l.y /= zi;
  geo.target_l.z = zb;

  // NED
  geo.x_t.x = pos->x - geo.target_l.x;
  geo.x_t.y = pos->y - geo.target_l.y;
  geo.x_t.z = 0;

  // ENU
  if (wp > 0) {
    waypoint_set_xy_i(wp, geo.x_t.y, geo.x_t.x);
    waypoint_set_alt_i(wp, geo.x_t.z);

    int32_t h = -geo.x_t.z;
    uint8_t wp_id = wp;
    DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &wp_id, &(geo.x_t.y),
                                   &(geo.x_t.x), &(h));

  }
}

void georeference_filter(bool kalman, int wp, int length)
{
  struct Int32Vect3 err;

  if (kalman)
  {
    // Predict
    VECT3_ADD(geo.filter.x, geo.filter.v);

    // Error = prediction - observation
    VECT3_COPY(err,geo.filter.x);
    VECT3_SUB(err, geo.x_t);
  }
  else // Average
  {
    VECT3_SMUL(geo.filter.x,geo.filter.x,geo.filter.P);
    VECT3_ADD(geo.filter.x, geo.x_t);
    geo.filter.P++;
    VECT3_SDIV(geo.filter.x,geo.filter.x,geo.filter.P);
    if (geo.filter.P > length) {
      geo.filter.P = length;
    }
  }

  // ENU
  waypoint_set_xy_i(wp, geo.filter.x.y, geo.filter.x.x);
  //waypoint_set_alt_i(wp, geo.filter.x.z);

  int32_t h = 0;
  uint8_t wp_id = wp;
  DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &wp_id, &(geo.filter.x.y),
                                 &(geo.filter.x.x), &(h));
}

int32_t focus_length;

void georeference_run(void)
{
  
  struct camera_frame_t target;
  if (global_filters[0].color_count) {
    target.w = 320;
    target.h = 240;
    target.f = focus_length;
    target.px = target.w/2 - global_filters[0].x_c;
    target.py = target.h/2 + global_filters[0].y_c;
    

    //georeference_project(&target, WP_DETECTED_OBJECT);
    //georeference_filter(FALSE, WP_DETECTED_OBJECT, 10);

    float nofly_before_x = waypoint_get_x(WP_bufferCenter);
    float nofly_before_y = waypoint_get_y(WP_bufferCenter);
    georeference_project(&target, WP_people);
    georeference_filter(FALSE, WP_people, 10);

   
    //use certain range to decide wheather to reconstract the graph 
    float nofly_detect_x = waypoint_get_x(WP_people);
    float nofly_detect_y = waypoint_get_y(WP_people);

    //second node 
    /*float nofly_before_x2 = waypoint_get_x(WP_bufferCenter2);
    float nofly_before_y2 = waypoint_get_y(WP_bufferCenter2);
    georeference_project(&target, WP_people2);
    georeference_filter(FALSE, WP_people2, 10);

   
    //use certain range to decide wheather to reconstract the graph 
    float nofly_detect_x2 = waypoint_get_x(WP_people2);
    float nofly_detect_y2 = waypoint_get_y(WP_people2);*/

    if ((nofly_detect_x - nofly_before_x < 5 &&
    nofly_detect_x - nofly_before_x > -5)) {
      printf("Judge no reconstructing, nofly_before_x %f, nofly_after_x %f\n", nofly_before_x, nofly_detect_x);
      return; //if it's in the range we don't reconstruct the graph
    }

//else we reconstruct the graph 
    
 
    /*if (WP_noflypoint > 0) {
    waypoint_set_xy_i(WP_noflypoint, nofly_detect_x, nofly_detect_y);
    waypoint_set_alt_i(WP_noflypoint, 0);

    int32_t h = 0;
    uint8_t wp_id = WP_noflypoint;
    DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &wp_id, &(nofly_detect_x),
                                   &(nofly_detect_y), &(h));

  }*/
    georeference_project(&target, WP_bufferCenter);
    georeference_filter(FALSE, WP_bufferCenter, 10);
    float xxx = waypoint_get_x(WP_bufferCenter);
    printf("shoulb be %f\n", nofly_detect_x);
    printf("noflypoint after detection:************************************* %f\n", xxx);

    /*georeference_project(&target, WP_bufferCenter2);
    georeference_filter(FALSE, WP_bufferCenter2, 10);*/

    
    float startX = GetPosX() * 253, startY = GetPosY() * 253;
    reconstruct_visibility_graph(HOME_NODE, startX, startY);
    //reconstruct the path node
    //printf("the dest_x %d, y %d \n", dest_x, dest_y);
    rebuild_path(startX, startY);


    /*temp_node = init_vis_node(GetPosX(), GetPosY(), vis_graph_size);
    for(int i = 1; i < vis_graph_size; i++) {
      if(is_visible2(temp_node, vis_graph_ref[i])) {
	add_neighbor(temp_node, vis_graph_ref[i], 0);
      }
    }
    struct vis_node *end_node = closest_node(HOME_NODE, dest_x, dest_y);
    free_path(PATH_START);
    PATH_START = astar_path(temp_node, end_node);
    CURR_NODE = PATH_START;*/
  } else {
    focus_length = 200;
  }
}

void georeference_init(void)
{
  INT32_VECT3_ZERO(geo.target_i);
  INT32_VECT3_ZERO(geo.target_l);

  VECT3_ASSIGN(geo.x_t, 0, 0, 0);

  INT32_VECT3_ZERO(geo.filter.v);
  INT32_VECT3_ZERO(geo.filter.x);
  geo.filter.P = 0;
  focus_length = 200;
}


