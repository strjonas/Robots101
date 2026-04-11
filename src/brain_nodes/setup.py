from setuptools import find_packages, setup


package_name = "brain_nodes"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jonas",
    maintainer_email="jonas@example.com",
    description="Runtime nodes for the robots101 control stack.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "cmd_arbiter_node = brain_nodes.cmd_arbiter_node:main",
            "cmd_vel_stamper_node = brain_nodes.cmd_vel_stamper_node:main",
            "control_supervisor_node = brain_nodes.control_supervisor_node:main",
            "observation_node = brain_nodes.observation_node:main",
            "object_tracker_node = brain_nodes.object_tracker_node:main",
            "localization_seed_node = brain_nodes.localization_seed_node:main",
            "patrol_node = brain_nodes.patrol_node:main",
            "planner_node = brain_nodes.planner_node:main",
            "safety_monitor_node = brain_nodes.safety_monitor_node:main",
            "skill_executor_node = brain_nodes.skill_executor_node:main",
            "task_server_node = brain_nodes.task_server_node:main",
            "submit_task_cli = brain_nodes.submit_task_cli:main",
            "clear_task_cli = brain_nodes.clear_task_cli:main",
            "set_emergency_cli = brain_nodes.set_emergency_cli:main",
            "record_waypoint_cli = brain_nodes.record_waypoint_cli:main",
        ],
    },
)
