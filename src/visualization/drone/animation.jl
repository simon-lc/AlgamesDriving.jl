################################################################################
# Animations
################################################################################

function animate_drone!(vis::Visualizer, anim::MeshCat.Animation, model::AbstractGameModel,
		s::AbstractVector; name::Symbol=:Robot)
	default_background!(vis)
	for t in 1:length(s)
		MeshCat.atframe(anim, t) do
			set_drone!(vis, model, s[t], name=name)
		end
	end
	setanimation!(vis, anim)
	return nothing
end

function visualize_drone!(vis::Visualizer, model::AbstractGameModel, s::AbstractVector;
		h=0.01, α=1.0,
		anim::MeshCat.Animation=MeshCat.Animation(Int(floor(1/h))),
		name::Symbol=:Robot)

	build_drone!(vis, model, name=name, α=α)
	animate_drone!(vis, anim, model, s, name=name)
	return anim
end

function visualize_drone!(vis::Visualizer, model::AbstractGameModel, traj::Algames.Traj;
		sample=max(1, Int(floor(traj[1].dt*length(traj) / 100))), h=traj[1].dt*sample, α=1.0,
		anim::MeshCat.Animation=MeshCat.Animation(Int(floor(1/h))),
		name::Symbol=:Robot)

	visualize_drone!(vis, model, Algames.state.(traj); anim=anim, name=name, h=h, α=α)
	return anim
end
