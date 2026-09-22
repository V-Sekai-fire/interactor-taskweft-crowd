@tool
extends Node3D

const MODEL := "res://plans/crowd.xml"

var _mj: MujocoWorld
var _holder: Node3D
var _meshes: Array = []

func _ready() -> void:
	_build_scene()
	_mj = MujocoWorld.new()
	_mj.model_path = MODEL
	_mj.substeps = 4
	add_child(_mj)
	_holder = Node3D.new()
	# MuJoCo is Z-up, Godot is Y-up.
	_holder.rotation = Vector3(-PI / 2.0, 0, 0)
	add_child(_holder)

func _process(_dt: float) -> void:
	if _mj == null or not _mj.alive():
		return
	# Steering stand-in for a taskweft agent: every pedestrian drives toward +x.
	var nu := _mj.nu()
	var ctrl := PackedFloat64Array()
	ctrl.resize(nu)
	for i in range(0, nu, 2):
		ctrl[i] = 1.0
	_mj.set_ctrl(ctrl)
	_mj.step()
	_draw_geoms()

func _draw_geoms() -> void:
	var g := _mj.geoms()
	var n := int(g.size() / 11.0)
	while _meshes.size() < n:
		var mi := MeshInstance3D.new()
		mi.material_override = StandardMaterial3D.new()
		_holder.add_child(mi)
		_meshes.append(mi)
	for i in range(n):
		var o := i * 11
		var t := int(g[o])
		var mi: MeshInstance3D = _meshes[i]
		if mi.mesh == null:
			mi.mesh = _mesh_for(t, g[o + 1], g[o + 2], g[o + 3])
			var m: StandardMaterial3D = mi.material_override
			m.albedo_color = Color(0.85, 0.7, 0.5) if t == 3 else Color(0.4, 0.42, 0.48)
			m.roughness = 0.7
		var xform := Transform3D()
		xform.basis = Basis(Quaternion(g[o + 8], g[o + 9], g[o + 10], g[o + 7]))
		if t == 0 or t == 3:
			xform.basis = xform.basis * Basis(Vector3(1, 0, 0), PI / 2.0)
		xform.origin = Vector3(g[o + 4], g[o + 5], g[o + 6])
		mi.transform = xform

func _mesh_for(t: int, sx: float, sy: float, sz: float) -> Mesh:
	match t:
		0:
			var p := PlaneMesh.new(); p.size = Vector2(max(sx, 1.0) * 2.0, max(sy, 1.0) * 2.0); return p
		2:
			var s := SphereMesh.new(); s.radius = sx; s.height = sx * 2.0; return s
		3:
			var c := CapsuleMesh.new(); c.radius = sx; c.height = sy * 2.0 + sx * 2.0; return c
	var b := BoxMesh.new(); b.size = Vector3(sx * 2.0, sy * 2.0, sz * 2.0); return b

func _build_scene() -> void:
	var we := WorldEnvironment.new()
	var env := Environment.new()
	env.background_mode = Environment.BG_COLOR
	env.background_color = Color(0.09, 0.1, 0.13)
	env.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	env.ambient_light_color = Color(0.5, 0.55, 0.65)
	env.ambient_light_energy = 0.7
	we.environment = env
	add_child(we)
	var light := DirectionalLight3D.new()
	light.rotation = Vector3(-1.1, 0.7, 0.0)
	add_child(light)
	var cam := Camera3D.new()
	cam.position = Vector3(0.0, 16.0, 15.0)
	cam.fov = 55.0
	add_child(cam)
	cam.look_at(Vector3(0.0, 0.0, 0.0), Vector3.UP)
