#
# Copyright (c) 2023 Composiv.ai
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0.
#
# SPDX-License-Identifier: EPL-2.0
#
# Contributors:
#   Composiv.ai - initial API and implementation
#

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription

pkg_name = "muto_core"
output = "screen"


def generate_launch_description():

    # Files
    file_core = os.path.join(get_package_share_directory("muto_core"), "config", "muto_core.yaml")

    # Nodes
    node_twin = Node(name="core_twin", package="muto_core", executable="twin", output=output, parameters=[file_core])

    # Launch Description Object
    ld = LaunchDescription()

    ld.add_action(node_twin)

    return ld
