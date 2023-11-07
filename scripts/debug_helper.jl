using ConstructionBots
using JLD2

"""
This script exists to helping debug scenarios. If a simulation doesn't progress after a
certain number of iterations, it saves the critical components of the simulation to a file.
This script can be used to load those "dumped" variables and continue the simulation,
enabling you to step through the simuation or other debugging techniques.
"""

dump_path = "variable_dump"
dump_name = "var_dump_3535.jld2"
file_dump_location = joinpath(dump_path, dump_name)


# Load variables from file
env = JLD2.load(file_dump_location, "env")
factory_vis = JLD2.load(file_dump_location, "factory_vis")
anim = JLD2.load(file_dump_location, "anim")
sim_params = JLD2.load(file_dump_location, "sim_params")
sim_process_data = JLD2.load(file_dump_location, "sim_process_data")

# Recreate rvo_sim
ConstructionBots.reset_rvo_python_module!()
ConstructionBots.rvo_set_new_sim!(ConstructionBots.rvo_new_sim())
scene_tree = env.scene_tree
ConstructionBots.rvo_add_agents!(scene_tree)

# Moddify the sim paramters as desired here
sim_batch_size = sim_params.sim_batch_size
max_time_steps = sim_params.max_time_steps
process_animation_tasks = sim_params.process_animation_tasks
save_anim_interval = sim_params.save_anim_interval
process_updates_interval = sim_params.process_updates_interval
update_anim_at_every_step = sim_params.update_anim_at_every_step
anim_active_agents = sim_params.anim_active_agents
anim_active_areas = sim_params.anim_active_areas
save_anim_prog_path = sim_params.save_anim_prog_path
save_animation_along_the_way = sim_params.save_animation_along_the_way
max_num_iters_no_progress = sim_params.max_num_iters_no_progress

sim_params = ConstructionBots.SimParameters(
    sim_batch_size,
    max_time_steps,
    process_animation_tasks,
    save_anim_interval,
    process_updates_interval,
    update_anim_at_every_step,
    anim_active_agents,
    anim_active_areas,
    save_anim_prog_path,
    save_animation_along_the_way,
    max_num_iters_no_progress
)

# Add debug options as desired. E.g. @run
ConstructionBots.run_simulation!(env, factory_vis, anim, sim_params)
