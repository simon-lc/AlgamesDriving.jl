using MeshCat
using REPL


function set_vis!(vis::Visualizer; color::AbstractVector=0.8*ones(3), α=0.5)
	name = "box"
	l = 0.1
    MeshCat.setobject!(
		vis[name*"/box"],
		MeshCat.HyperRectangle(MeshCat.Vec(l,l,l),MeshCat.Vec(2l,2l,2l)),
		MeshPhongMaterial(color=RGBA(color..., α)))
	return nothing
end

function set_state!(vis::Visualizer, x::AbstractVector)
	name = "box"
	MeshCat.settransform!(
		vis[name*"/box"],
		MeshCat.Translation(x...))
	return nothing
end


function play_traj!(vis::Visualizer, X::AbstractVector)
	for x in X
		set_state!(vis, x)
		sleep(0.02)
	end
	return nothing
end


vis = Visualizer()
open(vis)
set_vis!(vis)
x = [1, 0.5, 0.3]
set_state!(vis, x)
X = [[k*0.1, k*0.05, k*0.02] for k = 1:60]
play_traj!(vis, X)






function rr()
	term = REPL.Terminals.TTYTerminal("xterm",stdin,stdout,stderr)
	REPL.Terminals.raw!(term,true)
	Base.start_reading(stdin)

	count = 0
	char = 'N'
	while (true) && count < 200
		count += 1
	    bb = bytesavailable(stdin)
		@show bb
	    if bb > 0
	        data = read(stdin, bb)
	        if data[1] == UInt(3)
	            println("Ctrl+C - exiting")
	            exit()
	        end
	        println("Got $bb bytes: $(string(data))")
			char = Char.(data[end])
	    end
		@show char
		sleep(0.1)
	end
	return nothing
end
rr()




get_char()

function display_key()
	char = 'N'
	for k = 1:200
		c, empty_stream  = get_char()
		!empty_stream ? char = c : nothing
		@show char
		sleep(0.05)
	end
	return nothing
end
# display_key()

function play(vis::Visualizer)
	x = [0., 0., 0.]
	v = [0., 0., 0.]
	char = 'N'
	count = 0
	for k = 1:1000
		c, empty_stream  = get_char()
		if empty_stream
			count += 1
		else
			char = c
			count = 0
		end
		if count > 30
			char = 'N'
		end
		# @show char
		if char == 'N'
			v += [0., 0., 0.]
		elseif char == 'A'
			v += [0.01, 0., 0.]
		elseif char == 'B'
			v -= [0.01, 0., 0.]
		elseif char == 'D'
			v += [0., 0.01, 0.]
		elseif char == 'C'
			v -= [0., 0.01, 0.]
		end
		x += 10e-2*v
		v = 0.99*v
		set_state!(vis, x)
		sleep(0.00)
	end
	return nothing
end

@elapsed play(vis)

a = 10
a = 10
a = 10
a = 10
a = 10
