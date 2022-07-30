const assert = require("assert")
const path = require("path"),
	fs = require("fs")
const { vec2, vec3, vec4, quat, mat2, mat2d, mat3, mat4} = require("gl-matrix")

const glespath = path.join("..", "node-gles3")

const gl = require(path.join(glespath, "gles3.js")),
	glfw = require(path.join(glespath, "glfw3.js")),
	vr = require(path.join(glespath, "openvr.js")),
	glutils = require(path.join(glespath, "glutils.js"))

const sim = require('bindings')('an.node');


for (const event of ["exit", "SIGINT", "SIGUSR1", "SIGUSR2", "uncaughtException", "SIGTERM"]) {
	process.on(event, function() {
		console.log("goodbye!!", event)
		if (event != "exit") process.exit();
	})
}

function app() {
	if (!glfw.init()) {
		console.log("Failed to initialize GLFW");
		process.exit(-1);
	}
	let version = glfw.getVersion();
	console.log('glfw ' + version.major + '.' + version.minor + '.' + version.rev);
	console.log('glfw version-string: ' + glfw.getVersionString());

	// Open OpenGL window
	glfw.defaultWindowHints();
	glfw.windowHint(glfw.CONTEXT_VERSION_MAJOR, 3);
	glfw.windowHint(glfw.CONTEXT_VERSION_MINOR, 3);
	glfw.windowHint(glfw.OPENGL_FORWARD_COMPAT, 1);
	glfw.windowHint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE);

	let window = glfw.createWindow(720, 480, "Test");
	if (!window) {
		console.log("Failed to open GLFW window");
		glfw.terminate();
		process.exit(-1);
	}
	glfw.makeContextCurrent(window);
	console.log(gl.glewInit());

	//can only be called after window creation!
	console.log('GL ' + glfw.getWindowAttrib(window, glfw.CONTEXT_VERSION_MAJOR) + '.' + glfw.getWindowAttrib(window, glfw.CONTEXT_VERSION_MINOR) + '.' + glfw.getWindowAttrib(window, glfw.CONTEXT_REVISION) + " Profile: " + glfw.getWindowAttrib(window, glfw.OPENGL_PROFILE));

	// Enable vertical sync (on cards that support it)
	glfw.swapInterval(0); // 0 for vsync off
	glfw.pollEvents();

	// world spans from [0,0,0] to WORLD_DIM
	const WORLD_DIM = [6, 3, 6]
	const LIGHT_POS = [WORLD_DIM[0]/2, WORLD_DIM[0]*2, WORLD_DIM[0]/2]
	const NUM_POINTS = 20000;
	const NUM_GHOSTPOINTS = 320000;

	let vrdim = [4096, 2048]

	let quadProgram = glutils.makeProgram(gl,
		fs.readFileSync("shaders/q.vert", "utf-8"),
		fs.readFileSync("shaders/q.frag", "utf-8")
	);
	let wallProgram = glutils.makeProgram(gl,
		fs.readFileSync("shaders/w.vert", "utf-8"),
		fs.readFileSync("shaders/w.frag", "utf-8")
	);
	let beetleProgram = glutils.makeProgram(gl, 
		fs.readFileSync("shaders/b.vert", "utf-8"),
		fs.readFileSync("shaders/b.frag", "utf-8")
	);
	let snakeProgram = glutils.makeProgram(gl, 
		fs.readFileSync("shaders/s.vert", "utf-8"),
		fs.readFileSync("shaders/s.frag", "utf-8")
	);
	let pointsProgram = glutils.makeProgram(gl,
		fs.readFileSync("shaders/p.vert", "utf-8"),
		fs.readFileSync("shaders/p.frag", "utf-8")
	);
	let ghostProgram = glutils.makeProgram(gl,
		fs.readFileSync("shaders/g.vert", "utf-8"),
		fs.readFileSync("shaders/g.frag", "utf-8")
	);
	// let isoProgram = glutils.makeProgram(gl,
	// 	fs.readFileSync("shaders/i.vert", "utf-8"),
	// 	fs.readFileSync("shaders/i.frag", "utf-8")
	// );

	let fbo = glutils.makeFboWithDepth(gl, vrdim[0], vrdim[1], true)
	let quad = glutils.createVao(gl, glutils.makeQuad(), quadProgram.id);
	
	let wallVao = glutils.createVao(gl, glutils.makeCube({ min: [0,0,0], max:WORLD_DIM, div:WORLD_DIM }), wallProgram.id);

	let points = {
		vertices: new Float32Array(NUM_POINTS*3),
		colors: new Float32Array(NUM_POINTS*4),
	}
	for (let i=0; i<NUM_POINTS; i++) {
		vec3.set(new Float32Array(points.vertices.buffer, i*3 * points.vertices.BYTES_PER_ELEMENT, 3), 
			WORLD_DIM[0]*Math.random(), 
			WORLD_DIM[1]*Math.random(), 
			WORLD_DIM[2]*Math.random()
		);
		vec4.set(new Float32Array(points.colors.buffer, i*4 * points.colors.BYTES_PER_ELEMENT, 4), 
			Math.random(), 
			Math.random()*4, 
			Math.random()*4,
			Math.random()
		);
	}
	let pointsVao = glutils.createVao(gl, points, pointsProgram.id)

	let ghostVertices = new Float32Array(NUM_GHOSTPOINTS*4)
	for (let i=0; i<NUM_GHOSTPOINTS; i++) {
		vec4.set(new Float32Array(ghostVertices.buffer, i*3 * ghostVertices.BYTES_PER_ELEMENT, 4), 
			WORLD_DIM[0]*Math.random(), 
			WORLD_DIM[1]*Math.random(), 
			WORLD_DIM[2]*Math.random(),
			1
		);
	}
	let ghostVao = glutils.createVao(gl, {vertices:ghostVertices}, ghostProgram.id)

	let beetleGeom = glutils.geomAppend(glutils.geomFromOBJ(fs.readFileSync("objs/wingset01.obj", "utf-8")), glutils.geomFromOBJ(fs.readFileSync("objs/spc_highr_end03.obj", "utf-8")))
	let beetleVao = glutils.createVao(gl, beetleGeom, beetleProgram.id);
	let beetleInstances = glutils.createInstances(gl, [
		{ name:"i_quat", components:4 },
		{ name:"i_pos", components:3 },
		{ name:"i_age", components:1 },
		{ name:"i_scale", components:2 },
		{ name:"i_color", components:3 },
	]);
	beetleInstances.attachTo(beetleVao).allocate(2048)
	beetleInstances.count=beetleInstances.allocated;
	beetleInstances.instances.forEach(obj=>{
		vec3.set(obj.i_pos, 
			WORLD_DIM[0]*Math.random(), 
			WORLD_DIM[1]*Math.random(), 
			WORLD_DIM[2]*Math.random()
		);
		let s = 0.01
		vec2.set(obj.i_scale, s, s*0.8);

		quat.random(obj.i_quat)

		vec3.set(obj.i_color, 
			0.5/(Math.random()+0.5),
			0.4 + 0.1*Math.random(),
			0.5*(1 - 0.5/(Math.random()+0.5))
		)
	})
	
	let snakeGeom = glutils.geomFromOBJ(fs.readFileSync("objs/snake_fat_adjust1.obj", "utf-8"))
	let snakeVao = glutils.createVao(gl, snakeGeom, snakeProgram.id);
	let snakeInstances = glutils.createInstances(gl, [
		{ name:"i_quat", components:4 },
		{ name:"i_pos", components:3 },
		{ name:"i_phase", components:1 },
		{ name:"i_scale", components:2 },
		{ name:"i_color", components:1 },
	]);
	snakeInstances.attachTo(snakeVao).allocate(138)
	snakeInstances.count=snakeInstances.allocated;
	snakeInstances.instances.forEach(obj=>{
		vec3.set(obj.i_pos, 
			WORLD_DIM[0]*Math.random(), 
			WORLD_DIM[1]*Math.random(), 
			WORLD_DIM[2]*Math.random()
		);
		let s = 0.1
		vec2.set(obj.i_scale, s, s*0.8);

		quat.random(obj.i_quat)

		obj.i_color[0] = Math.random()
	})

	let t = glfw.getTime()
	let fps = 60
	let viewmatrix = mat4.create();
	let projmatrix = mat4.create();
	let modelmatrix = mat4.create();

	function shutdown() {
		// Close OpenGL window and terminate GLFW
		glfw.destroyWindow(window);
		glfw.terminate();
		process.exit(0);
	}

	function draw_scene() {
		gl.depthMask(false)

		gl.lineWidth(16)
		wallProgram.begin()
			.uniform("u_world_dim", WORLD_DIM)
			.uniform("u_viewmatrix", viewmatrix)
			.uniform("u_projmatrix", projmatrix)
		wallVao.bind().draw().unbind();
		wallProgram.end();

		
		gl.depthMask(true)

		// snakes, beetles (instanced)
		beetleProgram.begin()
			.uniform("u_viewmatrix", viewmatrix)
			.uniform("u_projmatrix", projmatrix)
			.uniform("u_lightpos", LIGHT_POS)
		beetleVao.bind().drawInstanced(beetleInstances.count).unbind();
		beetleProgram.end();

		snakeProgram.begin()
			.uniform("u_viewmatrix", viewmatrix)
			.uniform("u_projmatrix", projmatrix)
			.uniform("u_lightpos", LIGHT_POS)
		snakeVao.bind().drawInstanced(snakeInstances.count).unbind();
		snakeProgram.end();

		gl.enable(gl.BLEND);
		gl.blendFunc(gl.SRC_ALPHA, gl.ONE);
		gl.depthMask(false)

		// iso/goo
		// ghost
		ghostProgram.begin()
			.uniform("u_viewmatrix", viewmatrix)
			.uniform("u_projmatrix", projmatrix)
			.uniform("u_pixelSize", 10 * fbo.height/2048)
		ghostVao.bind().drawPoints().unbind();
		ghostProgram.end();
		// particles
		pointsProgram.begin()
			.uniform("u_viewmatrix", viewmatrix)
			.uniform("u_projmatrix", projmatrix)
			.uniform("u_pixelSize", 10 * fbo.height/2048)
		pointsVao.bind().drawPoints().unbind();
		pointsProgram.end();
		
		gl.disable(gl.BLEND);
		gl.depthMask(true)
	}

	function animate() {
		if(glfw.windowShouldClose(window) || glfw.getKey(window, glfw.KEY_ESCAPE)) {
			shutdown();
		} else {
			setImmediate(animate)
		}
		
		const t1 = glfw.getTime();
		const dt = t1-t;
		fps += 0.1*((1/dt)-fps);
		t = t1;

		// simulate

		// update instances
		beetleInstances.instances.forEach(obj=>{
			obj.i_age[0] = t;
		})
		snakeInstances.instances.forEach(obj=>{
			obj.i_phase[0] = t;
		})


		// Get window size (may be different than the requested size)
		let dim = glfw.getFramebufferSize(window);
		let centre = vec3.scale([0,0,0], WORLD_DIM, 0.5)
		let r = WORLD_DIM[2]/2
		let a = t * 0.1
		let eye = vec3.add([0,0,0], [r*Math.sin(a),0,r*Math.cos(a)], centre)

		mat4.lookAt(viewmatrix, eye, centre, [0, 1, 0]);
		mat4.perspective(projmatrix, Math.PI/2, dim[0]/dim[1], 0.01, 100);

		// submit buffers
		beetleInstances.bind().submit().unbind()
		snakeInstances.bind().submit().unbind()
		
		glfw.setWindowTitle(window, `fps ${fps}`);

		fbo.begin();
		{
			gl.viewport(0, 0, fbo.width, fbo.height);
			gl.enable(gl.DEPTH_TEST)
			gl.depthMask(true)
			gl.clearColor(0, 0, 0, 1);
			gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

			draw_scene()
		}
		fbo.end()
		//if (vr) vr.submit(fbo.colorTexture)
		gl.viewport(0, 0, dim[0], dim[1]);
		gl.enable(gl.DEPTH_TEST)
		gl.depthMask(true)
		gl.clearColor(0., 0., 0., 1);
		gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

		let f = 0.01
		quadProgram.begin()
			.uniform("bl_fade", [f,f])
			.uniform("tr_fade", [f,f])
			.uniform("tl", [0,0])
			.uniform("tr", [0,0])
			.uniform("bl", [0,0])
			.uniform("br", [0,0])
		quad.bind().draw().unbind();
		quadProgram.end();

		// Swap buffers
		glfw.swapBuffers(window);
		glfw.pollEvents();
	}

	animate()
}

try {

	app()
} catch(e) {
	console.log(e)
}