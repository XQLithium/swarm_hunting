---
trigger: always_on
---

- The project 'ego-planner-swarm' uses a lightweight simulator 'poscmd_2_odom' from 'uav_simulator/fake_drone' which subscribes to 'quadrotor_msgs/PositionCommand' and publishes 'nav_msgs/Odometry'. Visualization is handled by 'odom_visualization'. The swarm launch structure typically involves 'swarm.launch' calling 'run_in_sim.launch' for each drone.
- 我当前进行的工作是在ego-planner-swarm项目的基础上，部署AICG制导律。该制导律详情可见/home/qianli/ego-planner-swarm/summary_MD/paper_analysis_and_implementation_plan.md，这是总结文档，请你在我要求时在文档中更新当前算法部署的进度，并可以参考该文档的记录.要求使用中文进行记录，总结对哪些内容进行了更改，实现了什么新功能。
- AICG算法来源于文章/home/qianli/ego-planner-swarm/summary_MD/reference_paper.pdf，当搞不清楚算法细节时，应当查阅这个文档