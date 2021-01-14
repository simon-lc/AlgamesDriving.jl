################################################################################
# VehicleState
################################################################################

mutable struct VehicleState{T}
    x::T # x-position
    y::T # y-position
    θ::T # heading
    v::T # velocity
end

function VehicleState()
    x = 0.0
    y = 0.0
    θ = 0.0
    v = 0.0
    return VehicleState{typeof(x)}(x,y,θ,v)
end

import Base.==
import Base.+
import Base.*
import Base.copy

function (==)(x1::VehicleState{T}, x2::VehicleState{T}) where {T}
	out = true
	for name in fieldnames(VehicleState)
		out &= getfield(x1, name) == getfield(x2, name)
	end
	return out
end

function (+)(x1::VehicleState{T}, x2::VehicleState{T}) where {T}
	return VehicleState{T}(x1.x+x2.x, x1.y+x2.y, x1.θ+x2.θ, x1.v+x2.v)
end

function (*)(x1::VehicleState{T}, α::T) where {T}
	return VehicleState{T}(x1.x*α, x1.y*α, x1.θ*α, x1.v*α)
end

function (*)(α::T, x1::VehicleState{T}) where {T}
	return VehicleState{T}(x1.x*α, x1.y*α, x1.θ*α, x1.v*α)
end

function copy(x::VehicleState{T}) where {T}
	return VehicleState{T}(x.x, x.y, x.θ, x.v)
end

################################################################################
# standardize
################################################################################

function standardize(model::DoubleIntegratorGame{n,m,p}, x::SVector{n,T}) where {n,m,p,T}
    return [standardize(model, x[model.pz[i]]) for i=1:p]
end

function standardize(model::UnicycleGame{n,m,p}, x::SVector{n,T}) where {n,m,p,T}
    return [standardize(model, x[model.pz[i]]) for i=1:p]
end

function standardize(model::BicycleGame{n,m,p}, x::SVector{n,T}) where {n,m,p,T}
    return [standardize(model, x[model.pz[i]]) for i=1:p]
end

function standardize(model::DoubleIntegratorGame{n,m,p}, x::SVector{ni,T}) where {n,m,p,ni,T}
    vs = VehicleState()
    vs.x = x[1]
    vs.y = x[2]
    vs.θ = atan(x[4],x[3])
    vs.v = sqrt(x[3]^2+x[4]^2)
    return vs
end

function standardize(model::UnicycleGame{n,m,p}, x::SVector{ni,T}) where {n,m,p,ni,T}
    vs = VehicleState()
    vs.x = x[1]
    vs.y = x[2]
    vs.θ = x[3]
    vs.v = x[4]
    return vs
end

function standardize(model::BicycleGame{n,m,p}, x::SVector{ni,T}) where {n,m,p,ni,T}
    # x = [x,y,v,ψ]
    vs = VehicleState()
    vs.x = x[1]
    vs.y = x[2]
    vs.θ = x[4] # ψ
    vs.v = x[3]
    return vs
end


################################################################################
# specialize
################################################################################

function specialize(model::AbstractGameModel, vs::AbstractVector{VehicleState})
    p = length(vs)
    return [specialize(model, vs[i]) for i=1:p]
end

function specialize(model::UnicycleGame{n,m,p}, x::SVector{n,T}) where {n,m,p,T}
    return [specialize(model, x[model.pz[i]]) for i=1:p]
end

function specialize(model::BicycleGame{n,m,p}, x::SVector{n,T}) where {n,m,p,T}
    return [specialize(model, x[model.pz[i]]) for i=1:p]
end

function specialize(model::DoubleIntegratorGame{n,m,p}, vs::VehicleState{T}) where {n,m,p,T}
    x = @SVector [vs.x, vs.y, vs.v*cos(vs.θ), vs.v*sin(vs.θ)]
    return x
end

function specialize(model::UnicycleGame{n,m,p}, vs::VehicleState{T}) where {n,m,p,T}
    x = @SVector [vs.x, vs.y, vs.θ, vs.v]
    return x
end

function specialize(model::BicycleGame{n,m,p}, vs::VehicleState{T}) where {n,m,p,T}
    x = @SVector [vs.x, vs.y, vs.v, vs.θ]
    return x
end


@generated function specialize(model::DoubleIntegratorGame{n,m,p}, vs::AbstractVector{VehicleState{T}}) where {n,m,p,T}
	x = [:(vs[$i].x) for i=1:p]
	y = [:(vs[$i].y) for i=1:p]
	vx = [:(vs[$i].v*cos(vs[$i].θ)) for i=1:p]
	vy = [:(vs[$i].v*sin(vs[$i].θ)) for i=1:p]
	return :(SVector{$n}($(x...), $(y...), $(vx...), $(vy...)))
end

@generated function specialize(model::UnicycleGame{n,m,p}, vs::AbstractVector{VehicleState{T}}) where {n,m,p,T}
	x = [:(vs[$i].x) for i=1:p]
	y = [:(vs[$i].y) for i=1:p]
	θ = [:(vs[$i].θ) for i=1:p]
	v = [:(vs[$i].v) for i=1:p]
	return :(SVector{$n}($(x...), $(y...), $(θ...), $(v...)))
end

@generated function specialize(model::BicycleGame{n,m,p}, vs::AbstractVector{VehicleState{T}}) where {n,m,p,T}
	x = [:(vs[$i].x) for i=1:p]
	y = [:(vs[$i].y) for i=1:p]
	v = [:(vs[$i].v) for i=1:p]
	ψ = [:(vs[$i].θ) for i=1:p]
	return :(SVector{$n}($(x...), $(y...), $(v...), $(ψ...)))
end
