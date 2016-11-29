require 'orocos'
require 'vizkit'
require 'readline'

Orocos::CORBA.max_message_size = 12000000 # stucks if > than this

include Orocos
Orocos.initialize

# Content of these language variables must not be german, otherwise '.' and ','
# are mixed reading numbers and a bad_alloc error occurs loading the scenes.
ENV['LANG'] = 'C'
ENV['LC_NUMERIC'] = 'C'

Orocos.run  'traversability_explorer::Task' => 'trav',
            'trajectory_helpers::GoalGenerator' => 'pose',
            "valgrind" => false,
            'output' => nil,
            "wait" => 1000 do

    trav = TaskContext::get 'trav'
    trav.traversability_map_id = "trav_map"

    trav.traversability_map_scalex =  0.03
    trav.traversability_map_scaley =  0.03
    trav.filename = "data/dilation_0cm.txt"
    trav.robot_fov_a = 1.0
    trav.robot_fov_b = 1.5
    trav.robot_fov_l = 1.5
       
   # trav.filename = "data/dummy.txt"
   # trav.traversability_map_scalex =  1
   # trav.traversability_map_scaley =  1

    trav.configure

    pose = TaskContext::get 'pose'
    pose.apply_conf_file("../trajectory_helpers/config/trajectory_helpers::GoalGenerator.yml", ["default"])
    pose.configure

    pose.goal_pose.connect_to(trav.robot_pose)  

    trav.start
    pose.start

    t1 = Thread.new do
    while true do
            Readline::readline("Hit enter to generate a goal pose ... ")
            goal.trigger
        end
    end


    Vizkit.display trav.port("traversability_map")
    Vizkit.display pose.goal_pose 
    Vizkit.exec

    Readline::readline("Hit ENTER to stop")
end
