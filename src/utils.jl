# Run tests locally
using Pkg
Pkg.activate("/home/simon/.julia/dev/AlgamesPlots")
# Pkg.test("AlgamesPlots")


Pkg.activate("/home/simon/.julia/dev/AlgamesPlots/test")


function finite_difference_gradient(f, x; eps::T=1e-5)
	# the gradient dimension will be m = length(x)
	m = length(x)
	x = SVector{m,T}(x)
	grad = zeros(m)
	for k = 1:m
		epsk = zeros(m)
		epsk[k] = eps
		grad[k] = (f(x .+ epsk) - f(x .- epsk))/(2eps)
	end
	return grad
end

function finite_difference_hessian(f, x; eps::T=1e-5)
	# the Hessian dimension will be m.m = length(x).length(x)
	m = length(x)
	x = SVector{m,T}(x)
	hess = zeros(m,m)
	for k = 1:m
		for l = 1:m
			epsk = zeros(m)
			epsl = zeros(m)
			epsk[k] = eps
			epsl[l] = eps
			hess[k,l] = (f(x .+ epsk .+ epsl) - f(x .+ epsk .- epsl) - f(x .- epsk .+ epsl) + f(x .- epsk .- epsl))/(4eps^2)
		end
	end
	return hess
end

function finite_difference_jacobian(f, x; n::Int=length(f(x)), eps::T=1e-5)
	# n is the dimension of the output of f
	# the Jacobian dimension will be n.m = n.length(x)
	m = length(x)
	x = SVector{m,T}(x)
	jac = zeros(n,m)
	for k = 1:m
		epsk = zeros(m)
		epsk[k] = eps
		jac[:,k] = (f(x .+ epsk) - f(x .- epsk))/(2eps)
	end
	return jac
end
