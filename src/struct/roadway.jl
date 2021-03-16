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

################################################################################
# HighwayRoadwayOptions
################################################################################

@with_kw mutable struct HighwayRoadwayOptions{T} <: RoadwayOptions
    # Options
	"Length of the lanes."
	lane_length::T=4.0

	"Width of the lanes."
	lane_width::T=0.20
end

function build_roadway(opts::HighwayRoadwayOptions{T}) where {T}
	ll = opts.lane_length
	lw = opts.lane_width
	#     y
	#     ^          x3---------------------------------------------x2
	# 	  |							      w1
	# 	  |							      w2
	# (0,0) -->x     x4---------------------------------------------x1
	# 								      w3
	# 								      w4
	# 			     x5---------------------------------------------x6
	x1 = [ ll,  0.]
	x2 = [ ll,  lw]
	x3 = [ 0.,  lw]
	x4 = [ 0.,  0.]
	x5 = [ 0., -lw]
	x6 = [ ll, -lw]
	v = [0., 1.]
	w1 = Wall(x3, x2,  v)
	w2 = Wall(x4, x1, -v)
	w3 = Wall(x4, x1,  v)
	w4 = Wall(x5, x6, -v)

	# Centerlines
	f_left(s,v=0.)  = VehicleState(s,  lw/2, 0., v)
	f_right(s,v=0.) = VehicleState(s, -lw/2, 0., v)
	f_both(s,v=0.)  = VehicleState(s, -lw/2, 0., v)

	# Left lane
	left_lane = Lane(1,
					 :left_lane,
					 [w1,w2],
					 Vector{CircularWall}(),
					 StartingArea(
						 VehicleState(0.,  lw/2, 0.0,  0.00),
						 VehicleState(0.,  lw/2, 0.0, -0.05),
						 VehicleState(ll,  lw/2, 0.4,  0.05),
						 ),
					 f_left,
					 )

	# Right lane
	right_lane = Lane(2,
					  :right_lane,
					  [w3,w4],
					  Vector{CircularWall}(),
					  StartingArea(
						  VehicleState(0., -lw/2, 0.0,  0.00),
						  VehicleState(0., -lw/2, 0.0, -0.05),
					      VehicleState(ll, -lw/2, 0.4,  0.05),
					      ),
					  f_right,
					  )

	# Both lane
	both_lane = Lane(3,
					 :both_lane,
					 [w1,w4],
					 Vector{CircularWall}(),
					 StartingArea(
						 VehicleState(0., -lw/2, 0.0,  0.00),
						 VehicleState(0., -lw/2, 0.0, -0.05),
						 VehicleState(ll, -lw/2, 0.4,  0.05),
						 ),
					 f_both,
					 )

	lanes = [left_lane, right_lane, both_lane]
	roadway = Roadway(lanes, opts)
    return roadway
end

################################################################################
# MergingRoadwayOptions
################################################################################

@with_kw mutable struct MergingRoadwayOptions{T} <: RoadwayOptions
    # Options
	"Length of the lanes."
	lane_length::T=6.0

	"Width of the lanes."
	lane_width::T=0.20

	"Position of the incoming lane merging point."
	merging_point::T=1.2

	"Orientation of the incoming lane."
	merging_angle::T=pi/12
end

function build_roadway(opts::MergingRoadwayOptions{T}) where {T}
	ll = opts.lane_length
	lw = opts.lane_width
	mp = opts.merging_point
	θ  = opts.merging_angle
	#     y
	#     ^          x3---------------------------------------x-----x2
	# 	  |							  merging          -      x9       w1
	# 	  |							   point x7   -            x10     w2
	# (0,0) -->x     x4-----------------x-----------------------x---x1
	# 		    merging angle θ  -       \              -       w8  w3
	# 					  -               \      -      w7          w4
	# 		x11    - x5--------------------x------------------------x6
	#       -                       -      x8           w9
	#       w5               -      w6
	#                 -
	#                 x12
	x1  = [ ll,  0.]
	x2  = [ ll,  lw]
	x3  = [ 0.,  lw]
	x4  = [ 0.,  0.]
	x5  = [ 0., -lw]
	x6  = [ ll, -lw]

	x7  = [ mp,  0.]
	x8  = [ mp+lw*tan(θ), -lw]
	x9  = [ mp+lw/tan(θ),  lw]
	x10 = [ mp+lw*tan(θ)+lw/tan(θ), 0.]
	x11 = [ mp-mp*cos(θ), -mp*sin(θ)]
	x12 = [ mp+lw*tan(θ)-mp*cos(θ), -lw-mp*sin(θ)]

	v = [0., 1.]
	w = [-sin(θ), cos(θ)]
	w1 = Wall(x3, x2,  v)
	w2 = Wall(x4, x1, -v)
	w3 = Wall(x4, x1,  v)
	w4 = Wall(x5, x6, -v)

	w5 = Wall(x11, x7,   w)
	w6 = Wall(x12, x8,  -w)
	w7 = Wall(x12, x10, -w)
	@assert x10[1] < x1[1]
	w8 = Wall(x10, x1,  -v)
	@assert x8[1] < x1[1]
	w9 = Wall(x8, x6,   -v)

	# Centerlines
	f_straight_left(s,v=0.)  = VehicleState(s,  lw/2, 0., v)
	f_straight_right(s,v=0.) = VehicleState(s, -lw/2, 0., v)
	f_straight_both(s,v=0.)  = VehicleState(s, -lw/2, 0., v)
	function f_merging_left(s,v=0.)
		if s <= mp
			return VehicleState(mp+3lw/2*tan(θ)-(mp-s)*cos(θ), lw/2-(mp-s)*sin(θ), θ, v)
		else
			return VehicleState(3lw/2*tan(θ)+s,  lw/2, 0., v)
		end
	end
	function f_merging_right(s,v=0.)
		if s <= mp
			return VehicleState(mp+lw/2*tan(θ)-(mp-s)*cos(θ), -lw/2-(mp-s)*sin(θ), θ, v)
		else
			return VehicleState(lw/2*tan(θ)+s, -lw/2, 0., v)
		end
	end
	function f_merging_both(s,v=0.)
		if s <= mp
			return VehicleState(mp+lw/2*tan(θ)-(mp-s)*cos(θ), -lw/2-(mp-s)*sin(θ), θ, v)
		else
			return VehicleState(lw/2*tan(θ)+s, -lw/2, 0., v)
		end
	end


	# Straight Left lane
	straight_left_lane = Lane(1,
					 :straight_left_lane,
					 [w1,w2],
					 Vector{CircularWall}(),
					 StartingArea(
						 VehicleState(0.,  lw/2, 0.0,  0.00),
						 VehicleState(0.,  lw/2, 0.0, -0.05),
						 VehicleState(ll,  lw/2, 0.4,  0.05),
						 ),
					 f_straight_left,
					 )

	# Straight Right lane
	straight_right_lane = Lane(2,
					  :straight_right_lane,
					  [w3,w4],
					  Vector{CircularWall}(),
					  StartingArea(
						  VehicleState(0., -lw/2, 0.0,  0.00),
						  VehicleState(0., -lw/2, 0.0, -0.05),
					      VehicleState(ll, -lw/2, 0.4,  0.05),
					      ),
					  f_straight_right,
					  )

	# Straight Both lane
	straight_both_lane = Lane(3,
					 :straight_both_lane,
					 [w1,w4],
					 Vector{CircularWall}(),
					 StartingArea(
						 VehicleState(0., -lw/2, 0.0,  0.00),
						 VehicleState(0., -lw/2, 0.0, -0.05),
						 VehicleState(ll, -lw/2, 0.4,  0.05),
						 ),
					 f_straight_both,
					 )

 	# Merging Left lane
 	merging_left_lane = Lane(4,
 					 :merging_left_lane,
					 [w1,w5,w7,w8],
 					 Vector{CircularWall}(),
 					 StartingArea(
						 VehicleState(mp+lw/2*tan(θ)-mp*cos(θ), -lw/2-mp*sin(θ), 0.0, θ),
						 VehicleState(mp+lw/2*tan(θ)-mp*cos(θ), -lw/2-mp*sin(θ), 0.0, θ-0.05),
						 VehicleState(mp+lw/2*tan(θ),           -lw/2,           0.4, θ+0.05),
 						 ),
 					 f_merging_left,
 					 )

 	# Merging Right lane
 	merging_right_lane = Lane(5,
 					  :merging_right_lane,
					  [w3,w5,w6,w9],
 					  Vector{CircularWall}(),
 					  StartingArea(
						  VehicleState(mp+lw/2*tan(θ)-mp*cos(θ), -lw/2-mp*sin(θ), 0.0, θ),
						  VehicleState(mp+lw/2*tan(θ)-mp*cos(θ), -lw/2-mp*sin(θ), 0.0, θ-0.05),
						  VehicleState(mp+lw/2*tan(θ),           -lw/2,           0.4, θ+0.05),
 					      ),
 					  f_merging_right,
 					  )

 	# Merging Both lane
 	merging_both_lane = Lane(6,
 					 :merging_both_lane,
					 [w1,w5,w6,w9],
 					 Vector{CircularWall}(),
 					 StartingArea(
 						 VehicleState(mp+lw/2*tan(θ)-mp*cos(θ), -lw/2-mp*sin(θ), 0.0, θ),
 						 VehicleState(mp+lw/2*tan(θ)-mp*cos(θ), -lw/2-mp*sin(θ), 0.0, θ-0.05),
 						 VehicleState(mp+lw/2*tan(θ),           -lw/2,           0.4, θ+0.05),
 						 ),
 					 f_merging_both,
 					 )

	lanes = [straight_left_lane, straight_right_lane, straight_both_lane,
		merging_left_lane, merging_right_lane, merging_both_lane]
	roadway = Roadway(lanes, opts)
    return roadway
end


################################################################################
# IntersectionRoadwayOptions
################################################################################

@with_kw mutable struct FourIntersectionRoadwayOptions{T} <: RoadwayOptions
    # Options
	"Length of the lanes."
	lane_length::T=4.0

	"Width of the lanes."
	lane_width::T=0.4

	"Turn radius."
	turn_radius::T=0.1
end

function build_roadway(opts::FourIntersectionRoadwayOptions{T}) where {T}
	ll = opts.lane_length
	lw = opts.lane_width
	tr = opts.turn_radius
	#                  x5      x17       x6
	#                  |        .        |
	#                  |        .        |
	#                  |        .        |
	#                  |        .        |
	#            x29   x25      .        x26  x30
	#                  |        .        |
	# x1_________x21___|x9     x18      x10___x22___________x2
	#
	#                         (0,0)
	# x13..............x14	   x21     x15.................x16
	#
	#
	# x3_________x23___x11     x19      x12___x24___________x4
	#                  |        .        |
	#            x31   x27        .      x28  x32
	#                  |        .        |
	#                  |        .        |
	#                  |        .        |
	#                  |        .        |
	#                  |        .        |
	#                  x7      x20       x8
	#

	x1 = [ -ll/2, lw/2]
	x2 = [ ll/2, lw/2]
	x3 = [ -ll/2, -lw/2]
	x4 = [ ll/2, -lw/2]
	x5 = [ -lw/2, ll/2]
	x6 = [ lw/2, ll/2]
	x7 = [ -lw/2, -ll/2]
	x8 = [ lw/2, -ll/2]
	x9 = [ -lw/2, lw/2]
	x10 = [ lw/2, lw/2]
	x11 = [ -lw/2, -lw/2]
	x12 = [ lw/2, -lw/2]
	x13 = [ -ll/2, 0.]
	x14 = [ -lw/2, 0.]
	x15 = [ lw/2, 0.]
	x16 = [ ll/2, 0.]
	x17 = [ 0., ll/2]
	x18 = [ 0., lw/2]
	x19 = [ 0., -lw/2]
	x20 = [ 0., -ll/2]

	x21 = [-lw/2 - tr, +lw/2]
	x22 = [+lw/2 + tr, +lw/2]
	x23 = [-lw/2 - tr, -lw/2]
	x24 = [+lw/2 + tr, -lw/2]
	x25 = [-lw/2, +lw/2 + tr]
	x26 = [+lw/2, +lw/2 + tr]
	x27 = [-lw/2, -lw/2 - tr]
	x28 = [+lw/2, -lw/2 - tr]
	x29 = [-lw/2 - tr, +lw/2 + tr]
	x30 = [+lw/2 + tr, +lw/2 + tr]
	x31 = [-lw/2 - tr, -lw/2 - tr]
	x32 = [+lw/2 + tr, -lw/2 - tr]

	v = [0., 1.]
	w = [1., 0.]
    w1 = Wall(x1, x9, v)
	w2 = Wall(x10, x2, v)
	w3 = Wall(x3, x11, -v)
	w4 = Wall(x12, x4, -v)
	w5 = Wall(x9, x5, -w)
	w6 = Wall(x7, x11, -w)
	w7 = Wall(x8, x12,  w)
	w8 = Wall(x10, x6,  w)
	w9 = Wall(x13, x14, v)
	w10 = Wall(x13, x14, -v)
	w11 = Wall(x15, x16,  v)
	w12 = Wall(x15, x16, -v)
	w13 = Wall(x18, x17, -w)
	w14 = Wall(x18, x17,  w)
	w15 = Wall(x20, x19, -w)
	w16 = Wall(x20, x19,  w)

	# West
	trv = [tr, 0]
	trw = [0, tr]
	w101  = Wall(x3,  x4,  -v)
	w102  = Wall(x13, x16,  v)

	w103  = Wall(x3,  x23, -v)
	w104  = Wall(x13, x21,  v)
	w105  = Wall(x7,  x27, -w)
	w106  = Wall(x20, x21,  w)
	c107  = CircularWall(x31..., tr)

	w108  = Wall(x3,  x12, -v)
	w109  = Wall(x13, x21-trw,  v)
	w110  = Wall(x17, x21+trv, -w)
	w111  = Wall(x6 , x12,  w)
	c112  = CircularWall((x21-trw+trv)..., tr)

	f_west_straight(s,v=0.) = VehicleState(s-ll/2, -lw/4, 0., v)
	function f_west_right(s,v=0.)
		if s <= ll/2-lw/4
			vs = VehicleState(s-ll/2, -lw/4, 0., v)
		else
			vs = VehicleState(-lw/4, -lw/4-(s-(ll/2-lw/4)), -π/2, v)
		end
		return vs
	end
	function f_west_left(s,v=0.)
		if s <= ll/2+lw/4
			vs = VehicleState(s-ll/2, -lw/4, 0., v)
		else
			vs = VehicleState(+lw/4, -lw/4+(s-(ll/2+lw/4)), π/2, v)
		end
		return vs
	end
	f_east(s,v=0.) = VehicleState(ll/2 - s, lw/4, 3.14, v)
	f_north(s,v=0.) = VehicleState(-lw/4, s, 4.71, v)
	f_south(s,v=0.) = VehicleState(lw/4, ll/2 - s, 1.57, v)

	# <start_direction>_lane
	# West lane straight
	west_lane_straight = Lane(1,
					:west_lane,
					[w101, w102],
					Vector{CircularWall}(),
					StartingArea(
						VehicleState(-ll/2, -lw/4, 0.0,  0.00),
						VehicleState(-ll/2, -lw/4, 0.0, -0.05),
						VehicleState(-ll/2, -lw/4, 0.4,  0.05),
						),
					f_west_straight,
					)
	# West lane with right turn
	west_lane_right = Lane(5,
					:west_lane_right,
					[w103, w104, w105, w106],
					[c107],
					StartingArea(
						VehicleState(-ll/2, -lw/4, 0.0,  0.00),
						VehicleState(-ll/2, -lw/4, 0.0, -0.05),
						VehicleState(-ll/2, -lw/4, 0.4,  0.05),
						),
					f_west_right,
					)
	# West lane with left turn
	west_lane_left = Lane(5,
					:west_lane_left,
					[w108, w109, w110, w111],
					[c112],
					StartingArea(
						VehicleState(-ll/2, -lw/4, 0.0,  0.00),
						VehicleState(-ll/2, -lw/4, 0.0, -0.05),
						VehicleState(-ll/2, -lw/4, 0.4,  0.05),
						),
					f_west_left,
					)

	east_lane = Lane(2,
					:east_lane,
					[w1, w2, w10, w12],
					Vector{CircularWall}(),
					StartingArea(
						VehicleState(ll/2, lw/4, 3.14, 0.00),
						VehicleState(ll/2, lw/4, 3.14,-0.05),
						VehicleState(ll/2, lw/4, 3.14, 0.05),
						),
					f_east,
					)

	north_lane = Lane(3,
					:north_lane,
					[w5, w6, w14, w16],
					Vector{CircularWall}(),
					StartingArea(
						VehicleState(-lw/4, ll/2, 4.71,  0.00),
						VehicleState(-lw/4, ll/2, 4.71, -0.05),
						VehicleState(-lw/4, ll/2, 4.71,  0.05),
						),
					f_north,
					)

	south_lane = Lane(4,
					:south_lane,
					[w7, w8],
					Vector{CircularWall}(),
					StartingArea(
						VehicleState(lw/4,  -ll/2, 1.57,  0.00),
						VehicleState(lw/4,  -ll/2, 1.57, -0.05),
						VehicleState(lw/4,  -ll/2, 1.57,  0.05),
						),
					f_south,
					)

	lanes = [west_lane_straight, east_lane, north_lane, south_lane, west_lane_right, west_lane_left]
	roadway = Roadway(lanes, opts)
	return roadway
end
