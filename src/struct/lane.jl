################################################################################
# Circle
################################################################################

mutable struct Circle{T}
    x::T
    y::T
    r::T
end

################################################################################
# Lane
################################################################################

mutable struct Lane{T}
    id::Int
    wall::Vector{Wall}
    circle::Vector{Circle{T}}
    start::StartingArea{T}
    centerline::Function
end

function Lane(id::Int, wall::Vector{Wall}, circle::Vector{Circle}, start::StartingArea{T}, centerline) where {T}
    return Lane{T}(id, wall, circle, start, centerline)
end
