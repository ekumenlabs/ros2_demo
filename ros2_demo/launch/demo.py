from launch.exit_handler import restart_exit_handler


def launch(launch_descriptor, argv):
    ld = launch_descriptor
    ld.add_process(
        cmd=['driver'],
        name='driver_node',
        exit_handler=restart_exit_handler,
    )
    # ld.add_process(
    #     cmd=['astra_camera_node__rmw_opensplice_cpp'],
    #     name='astra_camera_node',
    #     exit_handler=restart_exit_handler,
    # )
    ld.add_process(
        cmd=['please_move'],
        name='please_mode_node',
        exit_handler=restart_exit_handler,
    )
    ld.add_process(
        cmd=['rosdi_ws'],
        name='rosdi_ws',
        exit_handler=restart_exit_handler,
    )
