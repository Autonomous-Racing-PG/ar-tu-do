-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

-- WARNING: we create a lot of X-Rays of a potentially large space in this
-- pipeline. For example, running over the
-- cartographer_paper_deutsches_museum.bag requires ~25GiB of memory. You can
-- reduce this by writing fewer X-Rays or upping VOXEL_SIZE - which is the size
-- of a pixel in a X-Ray.

-- Modified by the F1/10 Autonomous Racing Project Group

VOXEL_SIZE = 5e-2

XY_TRANSFORM =  {
  translation = { 0., 0., 0. },
  rotation = { 0., -math.pi / 2., 0., },
}

XZ_TRANSFORM =  {
  translation = { 0., 0., 0. },
  rotation = { 0. , 0., -math.pi / 2, },
}

YZ_TRANSFORM =  {
  translation = { 0., 0., 0. },
  rotation = { 0. , 0., math.pi, },
}

options = {
    tracking_frame = "base_link",
    pipeline = {
      {
        action = "min_max_range_filter",
        min_range = 0.1,
        max_range = 30.,
      },
      {
        action = "write_ros_map",
        range_data_inserter = {
          insert_free_space = true,
          hit_probability = 0.55,
          miss_probability = 0.49,
        },
        filestem = "map",
        resolution = 0.05,
      }
    }
  }

return options