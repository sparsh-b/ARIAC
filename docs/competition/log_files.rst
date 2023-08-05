.. _TRIAL_LOGS:

==================
Log Files
==================

Trial runs are logged to files located in :file:`~/.ariac2023/log/gazebo` and :file:`~/.ariac2023/log/scoring` directories. Each one of these folders will contain a subfolder for each trial run. The subfolder name is the name of the trial file. For instance, :file:`kitting.yaml` will be logged to :file:`~/.ariac2023/log/gazebo/kitting/state.log` and :file:`~/.ariac2023/log/scoring/kitting/scoring.log`

    - :file:`~/.ariac2023/log/gazebo/<trial_name>/state.log` - This file contains the Gazebo state of the world at each time step. This file is used to playback the trial run.

    .. admonition:: Generating state.log 
        :class: attention

        :file:`state.log` is only generated if the following entry is present in the trial file:

        .. code-block:: yaml

            gazebo_state_logging: true
        
    - :file:`~/.ariac2023/log/scoring/<trial_name>/scoring.log` - This file contains the scoring information for the trial run. The information in this file can also be seen in the terminal during the completion of the trial run. An example of the scoring log file is shown in the example below where the top part is the scoring information for each submitted order and the bottom part is a summary of the whole trial.

    .. code-block:: console
        :class: no-copybutton
        :caption: Example of a scoring log file
        
        ========================================
        Order: KITTING1
        Type: Kitting
        AGV: 3
        Tray score: 3
        Correct destination: Yes
        Bonus: 2
        Score: 11
        ========================================
        Quadrant: 1
        Quadrant score: 3
        ----------------------------------------
        Correct part type: Yes
        Correct part color: Yes
        Faulty part: No
        Flipped part: No
        ========================================
        Quadrant: 2
        Quadrant score: 3
        ----------------------------------------
        Correct part type: Yes
        Correct part color: Yes
        Faulty: No
        Flipped: No

        ========================================
        END OF TRIAL
        ========================================
        Trial file: my_kitting.yaml
        Trial time limit: 600.000000
        Trial completion time: 86.836000
        Trial score: 11
        ========================================
        ORDERS
        ========================================
        Kitting: KITTING1
        Announcement time: 0.000000
        Submission time: 86.836000
        Completion time: 86.836000
        Score: 11


Playing Back Gazebo State
-------------------------

To play back a specific :file:`state.log`, you will need to start  :file:`ariac_playback.launch.py` which is located in the folder :file:`ariac_plugins`, so make sure you have ARIAC installed and sourced.

:file:`ariac_playback.launch.py` will try to look for robot meshes and textures on the host machine the same way the robot model is fetched in the docker container, that is, in :file:`/home/ubuntu/ariac_ws/`. Since this path does not exist on the host, the launch file will crash. To solve this issue you can either create the path :file:`/home/ubuntu/ariac_ws/` on the host machine or you can create a symbolic link to the path where the robot meshes and textures are located on the host machine. 

    - Creating the path :file:`/home/ubuntu/ariac_ws/` on the host machine. If the robot meshes and textures are located in :file:`/home/john/my_ws/` on the host machine:

        .. code-block:: console

            sudo mkdir -p /home/ubuntu
            sudo cp -r /home/john/my_ws /home/ubuntu/ariac_ws

    - Creating a symbolic link to the path where the robot meshes and textures are located on the host machine.
    
            .. code-block:: console
    
                ln -s /home/john/my_ws /home/ubuntu/ariac_ws

Once you have created the path :file:`/home/ubuntu/ariac_ws/` on the host machine or created a symbolic link to the path where the robot meshes and textures are located on the host machine, you can start :file:`ariac_playback.launch.py`:

.. code-block:: console
    :caption: Starting ariac_playback.launch.py

    ros2 launch ariac_plugins ariac_playback.launch.py team_name:=nist_team trial_name:=final_kitting log_file:=state