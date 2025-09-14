package pinball

import "core:fmt"
import "core:math"
import b2 "vendor:box2d"
import rl "vendor:raylib"

// Constants
SCREEN_WIDTH :: 160
SCREEN_HEIGHT :: 144
BALL_RADIUS :: 2.0
FLIPPER_LENGTH :: 12.0
BUMPER_RADIUS :: 4.0

// Game state
GameState :: struct {
	world_id:             b2.WorldId,
	ball_body:            b2.BodyId,
	ball_shape:           b2.ShapeId,
	left_flipper:         b2.BodyId,
	right_flipper:        b2.BodyId,
	left_flipper_joint:   b2.JointId,
	right_flipper_joint:  b2.JointId,
	left_flipper_anchor:  b2.BodyId,
	right_flipper_anchor: b2.BodyId,
	bumpers:              [3]b2.BodyId,
	walls:                [dynamic]b2.BodyId,
	score:                int,
	ball_launched:        bool,
	plunger_power:        f32,
	camera:               rl.Camera2D,
}

game: GameState

main :: proc() {
	rl.InitWindow(SCREEN_WIDTH * 4, SCREEN_HEIGHT * 4, "Pinball")
	rl.SetTargetFPS(60)
	defer rl.CloseWindow()

	init_game()
	defer cleanup_game()

	for !rl.WindowShouldClose() {
		update_game()
		draw_game()
	}
}

init_game :: proc() {
	// Initialize Box2D world
	world_def := b2.DefaultWorldDef()
	world_def.gravity = {0, -20} // Downward gravity
	world_def.enableSleep = true
	game.world_id = b2.CreateWorld(world_def)

	// Setup camera for proper scaling
	game.camera = rl.Camera2D {
		offset   = {f32(SCREEN_WIDTH * 2), f32(SCREEN_HEIGHT * 2)},
		target   = {f32(SCREEN_WIDTH) / 2, f32(SCREEN_HEIGHT) / 2},
		rotation = 0.0,
		zoom     = 4.0,
	}

	create_walls()
	create_flippers()
	create_bumpers()
	create_ball()
}

cleanup_game :: proc() {
	b2.DestroyWorld(game.world_id)
	delete(game.walls)
}

create_walls :: proc() {
	// Wall creation helper
	create_wall :: proc(x1, y1, x2, y2: f32) -> b2.BodyId {
		body_def := b2.DefaultBodyDef()
		body_def.type = .staticBody
		body_def.position = {(x1 + x2) / 2, (y1 + y2) / 2}

		body_id := b2.CreateBody(game.world_id, body_def)

		shape_def := b2.DefaultShapeDef()
		shape_def.material.friction = 0.3
		shape_def.material.restitution = 0.6

		segment := b2.Segment {
			point1 = {x1 - body_def.position.x, y1 - body_def.position.y},
			point2 = {x2 - body_def.position.x, y2 - body_def.position.y},
		}

		shape_id := b2.CreateSegmentShape(body_id, shape_def, segment)
		return body_id
	}

	// Bottom wall (with gap for ball drain)
	append(&game.walls, create_wall(0, 0, 50, 0))
	append(&game.walls, create_wall(110, 0, SCREEN_WIDTH, 0))

	// Side walls
	append(&game.walls, create_wall(0, 0, 0, SCREEN_HEIGHT))
	append(&game.walls, create_wall(SCREEN_WIDTH, 0, SCREEN_WIDTH, SCREEN_HEIGHT))

	// Top wall
	append(&game.walls, create_wall(0, SCREEN_HEIGHT, SCREEN_WIDTH, SCREEN_HEIGHT))

	// Plunger lane wall
	append(&game.walls, create_wall(140, 0, 140, 80))

	// Flipper barriers
	append(&game.walls, create_wall(30, 25, 70, 35))
	append(&game.walls, create_wall(90, 35, 130, 25))
}

create_flippers :: proc() {
	// Left flipper anchor
	anchor_def := b2.DefaultBodyDef()
	anchor_def.type = .staticBody
	anchor_def.position = {40, 20}
	game.left_flipper_anchor = b2.CreateBody(game.world_id, anchor_def)

	// Left flipper
	flipper_def := b2.DefaultBodyDef()
	flipper_def.type = .dynamicBody
	flipper_def.position = {40 + FLIPPER_LENGTH / 2, 20}
	game.left_flipper = b2.CreateBody(game.world_id, flipper_def)

	shape_def := b2.DefaultShapeDef()
	shape_def.density = 1.0
	shape_def.material.friction = 0.8
	shape_def.material.restitution = 0.1

	// Flipper shape as capsule
	capsule := b2.Capsule {
		center1 = {-FLIPPER_LENGTH / 2, 0},
		center2 = {FLIPPER_LENGTH / 2, 0},
		radius  = 1.5,
	}
	_ = b2.CreateCapsuleShape(game.left_flipper, shape_def, capsule)

	// Left flipper joint
	joint_def := b2.DefaultRevoluteJointDef()
	joint_def.bodyIdA = game.left_flipper_anchor
	joint_def.bodyIdB = game.left_flipper
	joint_def.localAnchorA = {0, 0}
	joint_def.localAnchorB = {-FLIPPER_LENGTH / 2, 0}
	joint_def.enableLimit = true
	joint_def.lowerAngle = -0.5
	joint_def.upperAngle = 0.8
	joint_def.enableMotor = true
	joint_def.maxMotorTorque = 200.0
	game.left_flipper_joint = b2.CreateRevoluteJoint(game.world_id, joint_def)

	// Right flipper anchor
	anchor_def.position = {120, 20}
	game.right_flipper_anchor = b2.CreateBody(game.world_id, anchor_def)

	// Right flipper
	flipper_def.position = {120 - FLIPPER_LENGTH / 2, 20}
	game.right_flipper = b2.CreateBody(game.world_id, flipper_def)
	_ = b2.CreateCapsuleShape(game.right_flipper, shape_def, capsule)

	// Right flipper joint
	joint_def.bodyIdA = game.right_flipper_anchor
	joint_def.bodyIdB = game.right_flipper
	joint_def.localAnchorB = {FLIPPER_LENGTH / 2, 0}
	joint_def.lowerAngle = -0.8
	joint_def.upperAngle = 0.5
	game.right_flipper_joint = b2.CreateRevoluteJoint(game.world_id, joint_def)
}

create_bumpers :: proc() {
	bumper_positions := [3][2]f32{{50, 80}, {80, 95}, {110, 80}}

	for pos, i in bumper_positions {
		body_def := b2.DefaultBodyDef()
		body_def.type = .staticBody
		body_def.position = {pos.x, pos.y}

		game.bumpers[i] = b2.CreateBody(game.world_id, body_def)

		shape_def := b2.DefaultShapeDef()
		shape_def.material.restitution = 1.5 // Super bouncy
		shape_def.enableHitEvents = true

		circle := b2.Circle {
			center = {0, 0},
			radius = BUMPER_RADIUS,
		}

		_ = b2.CreateCircleShape(game.bumpers[i], shape_def, circle)
	}
}

create_ball :: proc() {
	body_def := b2.DefaultBodyDef()
	body_def.type = .dynamicBody
	body_def.position = {150, 40} // Start in plunger lane
	body_def.linearDamping = 0.1
	body_def.angularDamping = 0.1
	body_def.isBullet = true // Prevent tunneling

	game.ball_body = b2.CreateBody(game.world_id, body_def)

	shape_def := b2.DefaultShapeDef()
	shape_def.density = 1.0
	shape_def.material.friction = 0.3
	shape_def.material.restitution = 0.7
	shape_def.enableHitEvents = true

	circle := b2.Circle {
		center = {0, 0},
		radius = BALL_RADIUS,
	}

	game.ball_shape = b2.CreateCircleShape(game.ball_body, shape_def, circle)
}

update_game :: proc() {
	// Handle input
	handle_input()

	// Step physics
	time_step := rl.GetFrameTime()
	if time_step > 1.0 / 30.0 do time_step = 1.0 / 30.0 // Cap physics timestep
	b2.World_Step(game.world_id, time_step, 4)

	// Handle events
	handle_events()

	// Check if ball fell off bottom
	ball_pos := b2.Body_GetPosition(game.ball_body)
	if ball_pos.y < -10 {
		reset_ball()
	}
}

handle_input :: proc() {
	// Left flipper
	if rl.IsKeyDown(.LEFT) {
		b2.RevoluteJoint_SetMotorSpeed(game.left_flipper_joint, 15.0)
	} else {
		b2.RevoluteJoint_SetMotorSpeed(game.left_flipper_joint, -5.0)
	}

	// Right flipper  
	if rl.IsKeyDown(.RIGHT) {
		b2.RevoluteJoint_SetMotorSpeed(game.right_flipper_joint, -15.0)
	} else {
		b2.RevoluteJoint_SetMotorSpeed(game.right_flipper_joint, 5.0)
	}

	// Plunger
	if rl.IsKeyDown(.SPACE) && !game.ball_launched {
		game.plunger_power = min(game.plunger_power + rl.GetFrameTime() * 100, 100)
	} else if rl.IsKeyReleased(.SPACE) && !game.ball_launched {
		launch_ball()
	}
}

launch_ball :: proc() {
	if game.plunger_power > 0 {
		impulse := b2.Vec2{0, game.plunger_power * 0.3}
		b2.Body_ApplyLinearImpulseToCenter(game.ball_body, impulse, true)
		game.ball_launched = true
		game.plunger_power = 0
	}
}

reset_ball :: proc() {
	b2.Body_SetTransform(game.ball_body, {150, 40}, b2.MakeRot(0))
	b2.Body_SetLinearVelocity(game.ball_body, {0, 0})
	b2.Body_SetAngularVelocity(game.ball_body, 0)
	game.ball_launched = false
	game.plunger_power = 0
}

handle_events :: proc() {
	// Get collision events
	contact_events := b2.World_GetContactEvents(game.world_id)

	// Process hit events for scoring
	for i in 0 ..< contact_events.hitCount {
		hit := contact_events.hitEvents[i]

		// Check if ball hit a bumper
		for bumper in game.bumpers {
			if (hit.shapeIdA == game.ball_shape && b2.Shape_GetBody(hit.shapeIdB) == bumper) ||
			   (hit.shapeIdB == game.ball_shape && b2.Shape_GetBody(hit.shapeIdA) == bumper) {
				game.score += 100
				// Add extra impulse for dramatic effect
				ball_pos := b2.Body_GetPosition(game.ball_body)
				bumper_pos := b2.Body_GetPosition(bumper)
				direction := b2.Normalize(ball_pos - bumper_pos)
				impulse := 10.0 * direction
				b2.Body_ApplyLinearImpulseToCenter(game.ball_body, impulse, true)
				break
			}
		}
	}
}

draw_game :: proc() {
	rl.BeginDrawing()
	rl.ClearBackground(rl.BLACK)

	rl.BeginMode2D(game.camera)

	// Draw playfield background
	rl.DrawRectangle(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, rl.DARKGREEN)

	// Draw walls
	for wall in game.walls {
		draw_body_segments(wall, rl.WHITE)
	}

	// Draw flippers
	draw_body_capsule(game.left_flipper, rl.YELLOW)
	draw_body_capsule(game.right_flipper, rl.YELLOW)

	// Draw bumpers
	for bumper in game.bumpers {
		draw_body_circle(bumper, rl.RED)
	}

	// Draw ball
	draw_body_circle(game.ball_body, rl.WHITE)

	// Draw plunger power indicator
	if !game.ball_launched && game.plunger_power > 0 {
		bar_height := game.plunger_power * 0.3
		rl.DrawRectangle(155, 10, 3, i32(bar_height), rl.YELLOW)
	}

	rl.EndMode2D()

	// Draw UI
	rl.DrawText(rl.TextFormat("Score: %d", game.score), 10, 10, 20, rl.WHITE)
	rl.DrawText("LEFT/RIGHT: Flippers", 10, SCREEN_HEIGHT * 4 - 60, 16, rl.WHITE)
	rl.DrawText("SPACE: Launch Ball", 10, SCREEN_HEIGHT * 4 - 40, 16, rl.WHITE)

	if !game.ball_launched {
		rl.DrawText("Hold SPACE to charge plunger", 10, SCREEN_HEIGHT * 4 - 20, 16, rl.YELLOW)
	}

	rl.EndDrawing()
}

// Drawing helper functions
draw_body_circle :: proc(body_id: b2.BodyId, color: rl.Color) {
	pos := b2.Body_GetPosition(body_id)

	// Get the first shape to determine radius
	shape_id := b2.Body_GetShapes(body_id, nil, 1)[0]
	if b2.Shape_GetType(shape_id) == .circleShape {
		circle := b2.Shape_GetCircle(shape_id)
		rl.DrawCircle(i32(pos.x), i32(pos.y), circle.radius, color)
	}
}

draw_body_capsule :: proc(body_id: b2.BodyId, color: rl.Color) {
	pos := b2.Body_GetPosition(body_id)
	rot := b2.Body_GetRotation(body_id)

	//Get the first shape
	shape_id := b2.Body_GetShapes(body_id, nil, 1)[0]
	if b2.Shape_GetType(shape_id) == .capsuleShape {
		capsule := b2.Shape_GetCapsule(shape_id)

		//Transform capsule endpoints to world coordinates
		center1_world := pos + b2.RotateVector(rot, capsule.center1)
		center2_world := pos + b2.RotateVector(rot, capsule.center2)

		//Draw as thick line with rounded ends
		rl.DrawLineEx(
			{center1_world.x, center1_world.y},
			{center2_world.x, center2_world.y},
			capsule.radius * 2,
			color,
		)
	}
}

draw_body_segments :: proc(body_id: b2.BodyId, color: rl.Color) {
	pos := b2.Body_GetPosition(body_id)
	rot := b2.Body_GetRotation(body_id)

	// Get all shapes for this body
	shape_count := b2.Body_GetShapeCount(body_id)
	shapes := make([]b2.ShapeId, shape_count)
	defer delete(shapes)

	b2.Body_GetShapes(body_id, raw_data(shapes), shape_count)

	for shape_id in shapes {
		if b2.Shape_GetType(shape_id) == .segmentShape {
			segment := b2.Shape_GetSegment(shape_id)

			// Transform segment endpoints to world coordinates
			point1_world := pos + b2.RotateVector(rot, segment.point1)
			point2_world := pos + b2.RotateVector(rot, segment.point2)

			rl.DrawLineEx(
				{point1_world.x, point1_world.y},
				{point2_world.x, point2_world.y},
				2.0,
				color,
			)
		}
	}
}
