# The system: where the mission's data comes from

Every input to the mission in the sim, the nodes that produce it, and what runs inside the mission process. Read it left to right.

- **Grey:** Gazebo, where everything starts in the sim.
- **Blue:** our `perch_sim` stand-in nodes. Dashed ones (`mocap_sim`, `vio`) run, but the mission doesn't read them yet.
- **Orange:** PX4 SITL.
- **Dark:** the one mission process (`perch_mission`). Commander is its own node, `flight_commander`, but runs in the same process, and the mission calls its methods directly.

Labels on arrows between processes are topics or actions. Labels inside the mission process are method calls.

```mermaid
flowchart LR
    OP["Operator<br/>or sim_mission.sh"]

    subgraph GZ["Gazebo: perch_rod world"]
        TRUTH["Ground truth<br/>every link's pose"]
        GATE["Hook gate joint<br/>on x500_hook"]
        SENS["Sensors<br/>IMU, baro, flow,<br/>rangefinder"]
    end

    subgraph SIM["perch_sim nodes"]
        DET["branch_detector"]
        LEGS["legs_bridge<br/>(+ ros_gz_bridge)"]
        MOCAP["mocap_sim"]
        VIO["vio"]
    end

    subgraph PX["PX4 SITL"]
        PX4["PX4<br/>EKF, controllers,<br/>land detector"]
        AGENT["Micro XRCE-DDS<br/>agent"]
    end

    subgraph MISSION["mission process (perch_mission)"]
        ACTION["/vespr/perch<br/>action server"]
        LEVEL["on_branch()<br/>level(): roll, pitch out"]
        CYCLE["PerchCycle<br/>states, aborts"]
        PLAN["Planners<br/>fly_to · search · approach"]
        LCLIENT["Legs client"]
        CMD["Commander<br/>(perch_flight)"]
    end

    STATE(["/vespr/mission/state"])

    TRUTH -- "gz pose/info" --> DET
    TRUTH -.-> MOCAP
    TRUTH -.-> VIO
    GATE <-- "/hook/joint_state" --> LEGS
    SENS --> PX4
    PX4 <-- "uORB" --> AGENT

    OP -- "/vespr/perch goal" --> ACTION
    ACTION -- "goal" --> CYCLE
    DET -- "/vespr/perception/branch_pose" --> LEVEL
    AGENT -- "/fmu/out/vehicle_attitude" --> LEVEL
    LEVEL -- "levelled sighting" --> CYCLE
    CYCLE <-- "next step?" --> PLAN
    CYCLE -- "enable · grasp · release" --> LCLIENT
    LCLIENT <-- "/vespr/legs/*" --> LEGS
    CYCLE -- "takeoff · goto · move_body<br/>yaw_by · land · disarm" --> CMD
    CMD <-- "/fmu/in/* setpoints, commands<br/>/fmu/out/* position, status" --> AGENT
    CYCLE --> STATE

    classDef gz fill:#EAE6DD,stroke:#8A8F98,color:#15202B
    classDef sim fill:#FBFAF7,stroke:#2F6694,color:#15202B
    classDef unused fill:#FBFAF7,stroke:#8A8F98,color:#4B5563,stroke-dasharray:5 4
    classDef px4 fill:#FBFAF7,stroke:#A84A14,color:#15202B
    classDef mission fill:#1F2C3A,stroke:#F0A060,color:#F5F3EE
    classDef topic fill:none,stroke:none,color:#A84A14
    class TRUTH,GATE,SENS gz
    class DET,LEGS sim
    class MOCAP,VIO unused
    class PX4,AGENT px4
    class ACTION,LEVEL,CYCLE,PLAN,LCLIENT,CMD mission
    class STATE topic
    style GZ fill:#F3F0E8,stroke:#EAE6DD
    style SIM fill:none,stroke:#2F6694,stroke-dasharray:6 4
    style PX fill:none,stroke:#A84A14,stroke-dasharray:6 4
    style MISSION fill:#15202B,stroke:#15202B,color:#F0A060
```

On the real drone the left side changes, not the topics: a real detector instead of `branch_detector`, the ESP32 legs bridge instead of `legs_bridge`, and PX4 on the Pixhawk instead of SITL.
