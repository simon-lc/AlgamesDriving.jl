################################################################################
# Starting Area
################################################################################

mutable struct StartingArea{T}
    # This is a rectangular starting area on state space
    x_nom::VehicleState{T} # Nominal state
    x_min::VehicleState{T} # Bound on the starting state
    x_max::VehicleState{T} # Bound on the starting state
end

function StartingArea()
    return StartingArea(VehicleState(), VehicleState(), VehicleState())
end

function StartingArea(x_nom::VehicleState{T}) where {T}
    return StartingArea{T}(copy(x_nom), copy(x_nom), copy(x_nom))
end

function StartingArea(x_min::VehicleState{T}, x_max::VehicleState{T}) where {T}
    return StartingArea{T}(0.5*(x_min+x_max), x_min, x_max)
end

function randstate(start::StartingArea{T}) where {T}
    x_min = start.x_min
    x_max = start.x_max
    return VehicleState{T}(
        x_min.x + rand(T)*(x_max.x - x_min.x),
        x_min.y + rand(T)*(x_max.y - x_min.y),
        x_min.θ + rand(T)*(x_max.θ - x_min.θ),
        x_min.v + rand(T)*(x_max.v - x_min.v),
        )
end
