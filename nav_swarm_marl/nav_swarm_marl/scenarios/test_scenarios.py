from multi_robot_scenario import MultiRobotScenario

def main():
    file_path = "/media/asus/backup/zzzzz/ros2/rtw_workspaces/humble_ws/src/NavSwarmMARL/nav_swarm_marl/nav_swarm_marl/scenarios/multi_robot_scenario.yaml"
    scenario = MultiRobotScenario(file_path=file_path)
    # scenario.read_scenario(file_path)
    scenario.run_scenario()
    scenario.print_report()
    
if __name__ == "__main__":
    main()
