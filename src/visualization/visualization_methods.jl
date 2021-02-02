function clean!(vis::Visualizer)
    delete!(vis["/Grid"])
    delete!(vis["/Axes"])
    return nothing
end

function ModifiedMeshFileObject(obj_path::String, material_path::String; scale::T=1.0) where {T}
    obj = MeshFileObject(obj_path)
    rescaled_contents = rescale_contents(obj_path, scale=scale)
    material = select_material(material_path)
    mod_obj = MeshFileObject(
        rescaled_contents,
        obj.format,
        material,
        obj.resources,
        )
    return mod_obj
end

function rescale_contents(obj_path::String; scale::T=1.0) where T
    lines = readlines(obj_path)
    rescaled_lines = copy(lines)
    for (k,line) in enumerate(lines)
        if length(line) >= 2
            if line[1] == 'v'
                stringvec = split(line, " ")
                vals = []
                try
                    vals = map(x->parse(Float64,x),stringvec[end-2:end])
                catch e
                    vals = map(x->parse(Float64,x),stringvec[end-1:end])
                end
                rescaled_vals = vals .* scale
                rescaled_lines[k] = join([stringvec[1]; string.(rescaled_vals)], " ")
            end
        end
    end
    rescaled_contents = join(rescaled_lines, "\r\n")
    return rescaled_contents
end

function select_material(material_path::String)
    mtl_file = open(material_path)
    mtl = read(mtl_file, String)
    return mtl
end

function transRot(model::AbstractGameModel, x::SVx, i::Int) where {SVx}
    return transRot(standardize(model, x[model.pz[i]]))
end

function transRot(x::VehicleState)
    t = Translation(x.x, x.y, 0.)
    r = LinearMap(AngleAxis(x.θ, 0, 0, 1.))
    tr = compose(t,r)
    return compose(t,r)
end
