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
    trav.start

#    trav.traversability_map.connect_to(planner.traversability_map)
#    trav.start_state.connect_to(planner.start_state)
#    trav.goal_state.connect_to(planner.goal_state)


    Vizkit.display trav.port("traversability_map")
    Vizkit.exec

    Readline::readline("Hit ENTER to stop")
end
