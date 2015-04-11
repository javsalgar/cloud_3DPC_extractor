#!/usr/bin/env python
"""
  Cloud-based 3DPC Extraction platform Extractor Node

  @author Javier J. Salmeron-Garcia (jsalmeron2@us.es)

  University of Seville

  This file is part of Cloud-based 3DPC Extraction platform

  Cloud-based 3DPC Extraction platform is free software:
  you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Cloud-based 3DPC Extraction platform is distributed in the hope
  that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Cloud-based 3DPC Extraction platform.  If not, see <http://www.gnu.org/licenses/>.
"""

import rospy
from Extractor3DPC import *
import uuid

id = str(uuid.uuid4()).replace("-","")
rospy.init_node("wrapper_3dpc_extractor_" + id)

extractor_node = Extractor3DPC(id)
extractor_node.start()

rospy.spin()
