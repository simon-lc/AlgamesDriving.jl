################################################################################
# Roadway
################################################################################

abstract type RoadwayOptions
end

mutable struct Roadway{T}
    l::Int
    lane::Vector{Lane{T}}
	opts::RoadwayOptions
end

function Roadway(lanes::Vector{Lane{T}}, opts::RoadwayOptions) where {T}
    l = length(lanes)
    sort!(lanes, by = x -> x.id)
    if [lanes[i].id for i=1:l] != (1:l)
        # @show "Reindexing the lanes"
        for i = 1:l
            lanes[i].id = i
        end
    end
    return Roadway{T}(l,lanes,opts)
end

function add_lane!(roadway::Roadway{T}, lane::Lane{T}) where {T}
    roadway.l += 1
    lane.id = roadway.l
    push!(roadway.lane, lane)
    return nothing
end

function test_centerline_continuity(f; v::T=1.0, s_min::T=-10.0, s_max::T=10.0,
	sample::Int=2000) where T
	# plt = scatter(legend=false)
	out = true
	step = (s_max - s_min)/sample
	vs_ = f(s_min,v)
	vs = f(s_min,v)
	for k = 1:sample
		s = s_min + k*step
		vs = f(s,v)
		out &= norm([vs.x, vs.y] - [vs_.x, vs_.y]) <= (step + 1e-5)
		vs_ = vs
		# scatter!([vs.x], [vs.y])
	end
	# display(plt)
	return out
end
