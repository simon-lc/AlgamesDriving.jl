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
