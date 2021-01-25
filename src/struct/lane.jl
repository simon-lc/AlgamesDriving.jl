################################################################################
# Circle
################################################################################

mutable struct Circle{T}
    x::T
    y::T
    r::T
end

function Circle()
    x = 0.0
    y = 0.0
    r = 0.0
    return Circle{typeof(x)}(x,y,r)
end


################################################################################
# Lane
################################################################################

mutable struct Lane{T}
    id::Int
    name::Symbol
    wall::Vector{Wall}
    circle::Vector{Circle{T}}
    start::StartingArea{T}
    centerline::Function
end

function Lane()
    name = :lane_0
    w = Wall([0., 0.], [0., 0.], [0., 0.])
    c = Circle()
    start = StartingArea()
    return Lane{typeof(c.x)}(0, name, [w], [c], start, x -> x)
end

function Lane(id::Int, name::Symbol, wall::Vector{Wall}, circle::Vector{Circle}, start::StartingArea{T}, centerline) where {T}
    return Lane{T}(id, name, wall, circle, start, centerline)
end
