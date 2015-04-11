
/*********************************************************************************
 *
 *  Cloud-based 3DPC Extraction platform Front-end
 *
 *  @author Javier J. Salmeron-Garcia (jsalmeron2@us.es)
 *
 *  University of Seville
 *
 *  This file is part of Cloud-based 3DPC Extraction platform
 *
 *  Cloud-based 3DPC Extraction platform is free software:
 *  you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Cloud-based 3DPC Extraction platform is distributed in the hope
 *  that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Cloud-based 3DPC Extraction platform.  If not, see <http://www.gnu.org/licenses/>.
 *
 * *****************************************************************************/

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "cloud_3dpc_extractor/stereo_cam_buffer.h"

int main(int argc, char** argv) {
    
  ros::init(argc, argv, "stereo_cam_buffer");

  cloud_3DPC_extractor::StereoCamBuffer buffer;

  buffer.start();

  ros::spin();
}
