################################################################################
# FourIntersectionRoadwayOptions
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
	# x1_________x33___|x9     x18      x10___x22___________x2
	#
	#                         (0,0)
	# x13..............x14	   x21      x15.................x16
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

	x21 = [0., 0.]
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
	x33 = [-lw/2 - tr, +lw/2]

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
	w109  = Wall(x13, x21-trv,  v)
	w110  = Wall(x17, x21+trw, -w)
	w111  = Wall(x6 , x12,  w)
	c112  = CircularWall((x21-trw+trv)..., tr)

	# East
	w201  = Wall(x1,  x2,  v)
	w202  = Wall(x13, x16,  -v)

	w203  = Wall(x22,  x2, v)
	w204  = Wall(x16, x21,  -v)
	w205  = Wall(x26,  x6, w)
	w206  = Wall(x21, x17,  -w)
	c207  = CircularWall(x30..., tr)

	w208  = Wall(x9,  x2, v)
	w209  = Wall(x21+trv, x16, -v)
	w210  = Wall(x21-trw, x20,  w)
	w211  = Wall(x7 , x9,  -w)
	c212  = CircularWall((x21+trw-trv)..., tr)

	# North
	w301  = Wall(x5,  x7,  -w)
	w302  = Wall(x17, x20,  w)

	w303  = Wall(x25,  x5, -w)
	w304  = Wall(x17, x21,  w)
	w305  = Wall(x1,  x33, v)
	w306  = Wall(x21, x13,  -v)
	c307  = CircularWall(x29..., tr)

	w308  = Wall(x5,  x11, -w)
	w309  = Wall(x21+trw, x17, w)
	w310  = Wall(x21+trv, x16,  v)
	w311  = Wall(x4, x11,  -v)
	c312  = CircularWall((x21-trw-trv)..., tr)

	# South
	w401  = Wall(x6,  x8,  w)
	w402  = Wall(x17, x20,  -w)

	w403  = Wall(x8,  x28, w)
	w404  = Wall(x20, x21,  -w)
	w405  = Wall(x4,  x24, -v)
	w406  = Wall(x21, x16,  v)
	c407  = CircularWall(x32..., tr)

	w408  = Wall(x8,  x10,  w)
	w409  = Wall(x21-trw, x20, -w)
	w410  = Wall(x21-trv, x13,  -v)
	w411  = Wall(x1, x10,  v)
	c412  = CircularWall((x21+trw+trv)..., tr)

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

	f_east_straight(s,v=0.) = VehicleState(ll/2 - s, lw/4, 3.14, v)
	function f_east_right(s,v=0.)
		if s <= ll/2-lw/4
			vs = VehicleState(ll/2 - s, lw/4, 3.14, v)
		else
			vs = VehicleState(lw/4, lw/4+(s-(ll/2-lw/4)), π/2, v)
		end
		return vs
	end
	function f_east_left(s,v=0.)
		if s <= ll/2 + lw/4
			vs = VehicleState(ll/2 - s, lw/4, 3.14, v)
		else
			vs = VehicleState(-lw/4, lw/4-(s-(ll/2+lw/4)), -1.57, v)
		end
		return vs
	end
	f_north_straight(s,v=0.) = VehicleState(-lw/4, ll/2 - s, -1.57, v)
	function f_north_right(s,v=0.)
		if s <= ll/2-lw/4
			vs = VehicleState(-lw/4, ll/2 - s, -1.57, v)
		else
			vs = VehicleState(-lw/4, lw/4+(s-(ll/2-lw/4)), -π, v)
		end
		return vs
	end
	function f_north_left(s,v=0.)
		if s <= ll/2 + lw/4
			vs = VehicleState(-lw/4, ll/2 - s, -1.57, v)
		else
			vs = VehicleState(-lw/4, -lw/4+(s-(ll/2+lw/4)), -π, v)
		end
		return vs
	end
	f_south_straight(s,v=0.) = VehicleState(lw/4, s - ll/2, 1.57, v)
	function f_south_right(s,v=0.)
		if s <= ll/2-lw/4
			vs = VehicleState(lw/4, s - ll/2, 1.57, v)
		else
			vs = VehicleState(lw/4, -lw/4+(s-(ll/2-lw/4)), 0., v)
		end
		return vs
	end
	function f_south_left(s,v=0.)
		if s <= ll/2 + lw/4
			vs = VehicleState(lw/4, s - ll/2, 1.57, v)
		else
			vs = VehicleState(lw/4, +lw/4+(s-(ll/2+lw/4)), 3.14, v)
		end
		return vs
	end

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
	west_lane_right = Lane(2,
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
	west_lane_left = Lane(3,
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

	# East line straight
	east_lane_straight = Lane(4,
					:east_lane,
					[w201, w202],
					Vector{CircularWall}(),
					StartingArea(
						VehicleState(ll/2, lw/4, 3.14,  0.00),
						VehicleState(ll/2, lw/4, 3.10, -0.05),
						VehicleState(ll/2, lw/4, 3.18,  0.05),
						),
					f_east_straight,
					)
	# East lane with right turn
	east_lane_right = Lane(5,
					:east_lane_right,
					[w203, w204, w205, w206],
					[c207],
					StartingArea(
						VehicleState(ll/2, lw/4, 3.14,  0.00),
						VehicleState(ll/2, lw/4, 3.10, -0.05),
						VehicleState(ll/2, lw/4, 3.18,  0.05),
						),
					f_east_right,
					)
	# East lane with left turn
	east_lane_left = Lane(6,
					:east_lane_left,
					[w208, w209, w210, w211],
					[c212],
					StartingArea(
						VehicleState(ll/2, lw/4, 3.14,  0.00),
						VehicleState(ll/2, lw/4, 3.10, -0.05),
						VehicleState(ll/2, lw/4, 3.18,  0.05),
						),
					f_east_left,
					)
	# North lane straigth
	north_lane_straight = Lane(7,
					:north_lane,
					[w301, w302],
					Vector{CircularWall}(),
					StartingArea(
						VehicleState(-lw/4, ll/2, -1.57,  0.00),
						VehicleState(-lw/4, ll/2, -1.17, -0.05),
						VehicleState(-lw/4, ll/2, -1.97,  0.05),
						),
					f_north_straight,
					)
	# North lane with right turn
	north_lane_right = Lane(8,
					:north_lane_right,
					[w303, w304, w305, w306],
					[c307],
					StartingArea(
						VehicleState(-lw/4, ll/2, -1.57,  0.00),
						VehicleState(-lw/4, ll/2, -1.17, -0.05),
						VehicleState(-lw/4, ll/2, -1.97,  0.05),
						),
					f_north_right,
					)
	# North lane with left turn
	north_lane_left = Lane(9,
					:north_lane_left,
					[w308, w309, w310, w311],
					[c312],
					StartingArea(
						VehicleState(-lw/4, ll/2, -1.57,  0.00),
						VehicleState(-lw/4, ll/2, -1.17, -0.05),
						VehicleState(-lw/4, ll/2, -1.97,  0.05),
						),
					f_north_left,
					)
	# South lane straigth
	south_lane_straight = Lane(10,
					:south_lane,
					[w401, w402],
					Vector{CircularWall}(),
					StartingArea(
						VehicleState(lw/4, -ll/2, 1.57,  0.00),
						VehicleState(lw/4, -ll/2, 1.17, -0.05),
						VehicleState(lw/4, -ll/2, 1.97,  0.05),
						),
					f_south_straight,
					)
	# North lane with right turn
	south_lane_right = Lane(11,
					:north_lane_right,
					[w403, w404, w405, w406],
					[c407],
					StartingArea(
						VehicleState(lw/4, -ll/2, 1.57,  0.00),
						VehicleState(lw/4, -ll/2, 1.17, -0.05),
						VehicleState(lw/4, -ll/2, 1.97,  0.05),
						),
					f_north_right,
					)
	# North lane with left turn
	south_lane_left = Lane(12,
					:south_lane_left,
					[w408, w409, w410, w411],
					[c412],
					StartingArea(
						VehicleState(lw/4, -ll/2, 1.57,  0.00),
						VehicleState(lw/4, -ll/2, 1.17, -0.05),
						VehicleState(lw/4, -ll/2, 1.97,  0.05),
						),
					f_south_left,
					)

	lanes = [
		west_lane_straight, west_lane_right, west_lane_left,
		east_lane_straight, east_lane_right, east_lane_left,
		north_lane_straight, north_lane_right, north_lane_left,
		south_lane_straight, south_lane_right, south_lane_left]
	roadway = Roadway(lanes, opts)
	return roadway
end
