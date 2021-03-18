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
