using Plots
include("experiment_helpers.jl")

# Create Roadway
roadway_opts = MergingRoadwayOptions(merging_point=1.4, lane_width=0.28)
roadway = build_roadway(roadway_opts)

# Create players
T = Float64
p = 3
model = BicycleGame(p=p)
n = model.n
m = model.m

# Define the horizon of the problem
N = 15 # N time steps
dt = 0.10 # each step lasts 0.1 second
probsize = ProblemSize(N,model) # Structure holding the relevant sizes of the problem

# Define the objective of each player
# We use a LQR cost
Q = [Diagonal(3*SVector{model.ni[i],T}([1., 1.2, 1., 1.])) for i=1:p] # Quadratic state cost
R = [Diagonal(0.3*ones(SVector{model.mi[i],T})) for i=1:p] # Quadratic control cost
# Desrired state
xf = [SVector{model.ni[1],T}([2.0, 0.05, 0.0, 0]),
      SVector{model.ni[2],T}([2.0, 0.05, 0.0, 0]),
      SVector{model.ni[2],T}([2.0, 0.05, 0.0, 0]),
      ]
# Desired control
uf = [zeros(SVector{model.mi[i],T}) for i=1:p]
# Objectives of the game
game_obj = Algames.GameObjective(Q,R,xf,uf,N,model)
# radius = 0.22*ones(p)
# μ = 1.0*ones(p)
# add_collision_cost!(game_obj, radius, μ)

# Define the constraints that each player must respect
game_con = Algames.GameConstraintValues(probsize)
radius = 0.08
add_collision_avoidance!(game_con, radius)


# Define the initial state of the system
x0 = SVector{model.n,T}([
    0.2,     0.2,    0.2,
   -0.45,   -0.13,   0.13,
    0.45,    0.60,   0.60,
    0.15,    0.0,    0.0,
    ])

# Define the Options of the solver
opts = Options()
opts.ls_iter = 15
opts.outer_iter = 20
opts.inner_iter = 20
opts.ρ_0 = 1e0
opts.reg_0 = 1e-5
opts.α_dual = 1.0
opts.λ_max = 1.0*1e7
opts.ϵ_dyn = 1e-6
opts.ϵ_sta = 1e-6
opts.ϵ_con = 1e-6
opts.ϵ_opt = 1e-6
opts.regularize = true
# Define the game problem
prob = Algames.GameProblem(N,dt,x0,model,opts,game_obj,game_con)

# Solve the problem
newton_solve!(prob)

plot!(prob.model, prob.pdtraj.pr)
plot!(prob.stats)

players = Vector{Player{T}}(undef, p)
players[1] = Player(model, roadway.lane[3])
players[2] = Player(model, roadway.lane[3])
players[3] = Player(model, roadway.lane[3])

# Create Scenario
sce = Scenario(model, roadway, players)

# Initialize visualizers
vis = Visualizer()
open(vis)

# Visualize trajectories
set_scenario!(vis, sce)
set_env!(vis, VehicleState(0.9, -0.2, 0.0, 0.0))
set_camera_birdseye!(vis, height=3.5)

build_waypoint!(vis, sce.player, N, key=0)
set_waypoint_traj!(vis, model, sce, prob.pdtraj.pr, key=0)
set_traj!(vis, model, sce, prob.pdtraj.pr)
set_line_traj!(vis, model, sce.player, prob.pdtraj.pr, vis_opts=LineVisualizationOptions(line_width=8.0))

get_num_active_constraint(prob)
prob_copy = deepcopy(prob)
ascore = ActiveSetCore(probsize)
active_vertical_mask!(ascore, prob_copy.game_con)
vmask = deepcopy(ascore.vmask)


function get_figure(prob::Algames.GameProblem)
	probize = prob.probsize
	ascore = ActiveSetCore(probsize)
	update_nullspace!(ascore, prob, prob.pdtraj)
	ns = length(ascore.null.vec)
	N = prob.probsize.N
	n = prob.probsize.n
	m = prob.probsize.m
	p = prob.probsize.p
	S = prob.probsize.S
	Sh = S + p*(p-1)*(N-1)
	Sv = S + Int(p*(p-1)*(N-1)/2)
	x0 = get_primal_dual(ascore, prob)
	v0 = deepcopy(ascore.null.vec[2])

 	pdtrajs = []

	plt = plot()
	for l = 1:100
		ascore = ActiveSetCore(probsize)
		active_vertical_mask!(ascore, prob.game_con)
		vmask = deepcopy(ascore.vmask)
		na = get_num_active_constraint(prob)

		β = 1.0
		α = 1e-3
		λ = 1e-10*rand(Sv)
		x = get_primal_dual(ascore, prob)

		xp, λ = simple_projection(α, λ, x, v0, β, ascore, prob, vmask)
		set_primal_dual!(ascore, prob, xp)
		push!(pdtrajs, deepcopy(prob.pdtraj))

		plot!(model, prob.pdtraj.pr, plt=plt)
		prob_c = deepcopy(prob)
		set_primal_dual!(ascore, prob_c, x+α*v0)
	end
	display(plt)

	plt = plot()
	set_primal_dual!(ascore, prob, x0)
	plot!(model, prob.pdtraj.pr, plt=plt)
	set_primal_dual!(ascore, prob, x0+100*1e-3*v0)
	plot!(model, prob.pdtraj.pr, plt=plt)
	display(plt)

	return prob, pdtrajs
end

prob_copy = deepcopy(prob)
prob_copy, pdtrajs = get_figure(prob_copy)

for (k,pdtraj) in enumerate(pdtrajs[[1,20,40,60,80,100]])
	vis_opts = LineVisualizationOptions(line_width=2.0, α=1.0)
	set_line_traj!(vis, model, sce.player, pdtraj.pr, vis_opts=vis_opts, key=k)
end

settransform!(vis["env/roadway"], Translation(0.0, -0.05, 0.0))






# for (k,pdtraj) in enumerate(pdtrajs[[(1:1:80); (80:-1:1)]])
for (k,pdtraj) in enumerate(pdtrajs[[(1:1:80); (80:-1:1)]])
	set_traj!(vis, model, sce, pdtraj.pr[end:end])
	set_line_traj!(vis, model, sce.player, pdtraj.pr, vis_opts=LineVisualizationOptions(line_width=8.0))
	sleep(0.3)
	save_image(vis)
end

prob.pdtraj.pr

[(1:5:100);]
[(1:5:100);]
plot([(1:5:100); (100:-5:1)])


using FFMPEG
using MeshCat
filename = "GNE_continuum"
MeshCat.convert_frames_to_video(
# convert_frames_to_video(
    "/home/simon/Downloads/$filename.tar",
    "/home/simon/Documents/$filename.mp4", overwrite=true)

convert_video_to_gif(
    "/home/simon/Documents/$filename.mp4",
    "/home/simon/Documents/$filename.gif", overwrite=true)

unpack_cmd = MeshCat.unpack_cmd
using FFMPEG

function convert_frames_to_video(tar_file_path::AbstractString, output_path::AbstractString="output.mp4"; framerate=60, overwrite=false)
    output_path = abspath(output_path)
    if !isfile(tar_file_path)
        error("Could not find the input file $tar_file_path")
    end
    if isfile(output_path) && !overwrite
        error("The output path $output_path already exists. To overwrite that file, you can pass `overwrite=true` to this function")
    end

    mktempdir() do tmpdir
        run(unpack_cmd(tar_file_path, tmpdir, ".zip", nothing))
        cmd = ["-r", string(framerate), "-i", "*.png", "-vcodec", "libx264", "-preset", "slow", "-crf", "18"]
        if overwrite
            push!(cmd, "-y")
        end
        push!(cmd, output_path)

        cd(tmpdir) do
            FFMPEG.exe(cmd...)
        end
    end
    @info("Saved output as $output_path")
    return output_path
end




function convert_video_to_gif(video_file_path::AbstractString, output_path::AbstractString="output.gif";
    framerate::Int=30, start_time=0., duration=1e3, overwrite=false, width::Int=1080, height::Int=-2, hq_colors::Bool=false)
    output_path = abspath(output_path)

    if !isfile(video_file_path)
        error("Could not find the input file $video_file_path")
    end
    if isfile(output_path) && !overwrite
        error("The output path $output_path already exists. To overwrite that file, you can pass `overwrite=true` to this function")
    end

    mktempdir() do tmpdir
        # run(MeshCat.unpack_cmd(video_file_path, tmpdir, ".mp4", nothing)) # unpack the .tar file
        # cmd = ["-r", string(framerate), "-i", "%07d.png", "-vcodec", "libx264", "-preset", "slow", "-crf", "18"]
        color_map = hq_colors ?
            "[0:v] fps=$framerate, scale=$width:$height,split [a][b];[a] palettegen=stats_mode=single [p];[b][p] paletteuse=new=1" :
            "[0:v] fps=$framerate, scale=$width:$height,split [a][b];[a] palettegen [p];[b][p] paletteuse"
        cmd = ["-ss", string(start_time), "-t", string(duration), "-i", video_file_path, "-filter_complex", color_map]
        if overwrite
            push!(cmd, "-y")
        end
        push!(cmd, output_path)

        cd(tmpdir) do
            FFMPEG.exe(cmd...)
        end
    end
    @info("Saved output as $output_path")
    return output_path
end
