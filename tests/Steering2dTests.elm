module Steering2dTests exposing (suite)

import Acceleration
import Angle exposing (Angle)
import AngularAcceleration
import AngularSpeed exposing (AngularSpeed)
import Direction2d exposing (Direction2d)
import Duration exposing (Duration)
import Expect
import Fuzz exposing (Fuzzer)
import Length
import LineSegment2d
import Point2d exposing (Point2d)
import Quantity exposing (Quantity)
import Random
import Speed
import Steering2d
    exposing
        ( Collision2d
        , Kinematic2d
        , Steering2d
        , SteeringConfig2d
        , WanderConfig2d
        , accelerate
        , arrive
        , decelerate
        , flee
        , lookAt
        , lookWhereYoureGoing
        , none
        , rotate
        , rotateCounterclockwise
        , seek
        , stopRotating
        , wallAvoidance
        , wander
        )
import Test
    exposing
        ( Test
        , describe
        , fuzz
        , fuzz2
        , fuzz3
        , test
        )
import Vector2d exposing (Vector2d)


defaultConfig : SteeringConfig2d
defaultConfig =
    { minVelocity = Speed.metersPerSecond -5
    , maxVelocity = Speed.metersPerSecond 11.1
    , maxAcceleration = Acceleration.metersPerSecondSquared 5
    , maxDeceleration = Acceleration.metersPerSecondSquared -20
    , maxAngularAcceleration = AngularAcceleration.radiansPerSecondSquared 22.3
    , maxRotation = AngularSpeed.radiansPerSecond 2.5
    }


defaultWanderConfig : WanderConfig2d
defaultWanderConfig =
    { distance = Length.meters 3
    , radius = Length.meters 0.5
    , rate = 0.1
    }


defaultSeed : Random.Seed
defaultSeed =
    Random.initialSeed 42


type TestCoordinates
    = TestCoordinates


atOrigin : Kinematic2d TestCoordinates
atOrigin =
    { position = Point2d.origin
    , orientation = Quantity.zero
    , velocity = Vector2d.zero
    , rotation = Quantity.zero
    }


withPosition : Point2d Length.Meters TestCoordinates -> Kinematic2d TestCoordinates -> Kinematic2d TestCoordinates
withPosition position kinematic =
    { kinematic | position = position }


withOrientation : Angle -> Kinematic2d TestCoordinates -> Kinematic2d TestCoordinates
withOrientation orientation kinematic =
    { kinematic | orientation = orientation }


withVelocity : Vector2d Speed.MetersPerSecond TestCoordinates -> Kinematic2d TestCoordinates -> Kinematic2d TestCoordinates
withVelocity velocity kinematic =
    { kinematic | velocity = velocity }


withRotation : AngularSpeed -> Kinematic2d TestCoordinates -> Kinematic2d TestCoordinates
withRotation rotation kinematic =
    { kinematic | rotation = rotation }


point2dFuzzer : Fuzzer (Point2d Length.Meters TestCoordinates)
point2dFuzzer =
    Fuzz.map2 Point2d.meters
        (Fuzz.floatRange -100 100)
        (Fuzz.floatRange -100 100)


velocityVector2dFuzzer : Fuzzer (Vector2d Speed.MetersPerSecond TestCoordinates)
velocityVector2dFuzzer =
    Fuzz.map2 Vector2d.metersPerSecond
        (Fuzz.floatRange -10 10)
        (Fuzz.floatRange -10 10)


kinematicFuzzer : Fuzzer (Kinematic2d TestCoordinates)
kinematicFuzzer =
    Fuzz.map4
        Kinematic2d
        point2dFuzzer
        velocityVector2dFuzzer
        (Fuzz.map Angle.radians (Fuzz.floatRange -3.14 3.14))
        (Fuzz.map AngularSpeed.radiansPerSecond (Fuzz.floatRange -5 5))


angleFuzzer : Fuzzer Angle
angleFuzzer =
    Fuzz.map Angle.radians (Fuzz.floatRange -6.28 6.28)


seedFuzzer : Fuzzer Random.Seed
seedFuzzer =
    Fuzz.map Random.initialSeed (Fuzz.intRange 0 1000000)


closenessThreshold : Length.Length
closenessThreshold =
    Length.meters 0.1


suite : Test
suite =
    describe "Steering2d behaviors"
        [ basicsSmokeTests
        , lookWhereYoureGoingTests
        , lookWhereYoureGoingIntegrationTests
        , lookAtTests
        , lookAtFuzzTests
        , lookAtIntegrationTests
        , seekTests
        , seekFuzzTests
        , seekIntegrationTests
        , arriveTests
        , arriveFuzzTests
        , arriveIntegrationTests
        , wanderTests
        , wanderFuzzTests
        , wanderIntegrationTests
        , wallAvoidanceTests
        , wallAvoidanceFuzzTests
        , wallAvoidanceIntegrationTests
        ]


basicsSmokeTests : Test
basicsSmokeTests =
    describe "basic movement behaviors"
        [ accelerateTests
        , decelerateTests
        , rotateTests
        , stopRotatingTests
        ]


accelerateTests : Test
accelerateTests =
    describe "accelerate behavior"
        [ test "produces max acceleration" <|
            \_ ->
                let
                    config =
                        defaultConfig
                in
                accelerate config atOrigin
                    |> .linear
                    |> Maybe.map Vector2d.length
                    |> Expect.equal (Just config.maxAcceleration)
        , test "produces no angular acceleration" <|
            \_ ->
                let
                    config =
                        defaultConfig
                in
                accelerate config atOrigin
                    |> .angular
                    |> Expect.equal Nothing
        , test "increases velocity over time" <|
            \_ ->
                let
                    initialKinematic =
                        atOrigin |> withVelocity (Vector2d.withLength (Speed.metersPerSecond 2) Direction2d.positiveX)

                    trajectory =
                        simulateSteps 10 (\kinematic -> accelerate defaultConfig kinematic) initialKinematic

                    velocities =
                        trajectory
                            |> List.map (.kinematic >> .velocity >> Vector2d.length)

                    initialVelocity =
                        velocities
                            |> List.head
                            |> Maybe.withDefault Quantity.zero

                    finalVelocity =
                        velocities
                            |> List.reverse
                            |> List.head
                            |> Maybe.withDefault Quantity.zero
                in
                finalVelocity |> expectGreaterThan initialVelocity
        ]


decelerateTests : Test
decelerateTests =
    describe "decelerate behavior"
        [ test "produces deceleration when moving forward" <|
            \_ ->
                let
                    config =
                        defaultConfig

                    source =
                        atOrigin |> withVelocity (Vector2d.metersPerSecond 5 0)
                in
                case decelerate config source |> .linear of
                    Just acceleration ->
                        Vector2d.dot acceleration source.velocity
                            |> expectLessThan Quantity.zero

                    _ ->
                        Expect.fail "Expected linear acceleration"
        , test "produces acceleration opposing velocity direction (backwards velocity direction)" <|
            \_ ->
                let
                    config =
                        defaultConfig

                    source =
                        atOrigin |> withVelocity (Vector2d.metersPerSecond -5 0)
                in
                case decelerate config source |> .linear of
                    Just acceleration ->
                        Vector2d.dot acceleration source.velocity
                            |> expectLessThan Quantity.zero

                    _ ->
                        Expect.fail "Expected linear acceleration"
        , test "produces no angular acceleration" <|
            \_ ->
                let
                    config =
                        defaultConfig
                in
                decelerate config atOrigin
                    |> .angular
                    |> Expect.equal Nothing
        , test "reduces speed over time" <|
            \_ ->
                let
                    initialTransform =
                        atOrigin |> withVelocity (Vector2d.metersPerSecond 8 0)

                    trajectory =
                        simulateSteps 15 (\kinematic -> decelerate defaultConfig kinematic) initialTransform

                    speeds =
                        trajectory
                            |> List.map (.kinematic >> .velocity >> Vector2d.length)

                    initialSpeed =
                        speeds
                            |> List.head
                            |> Maybe.withDefault Quantity.zero

                    finalSpeed =
                        speeds
                            |> List.reverse
                            |> List.head
                            |> Maybe.withDefault Quantity.zero
                in
                finalSpeed |> expectLessThan initialSpeed
        ]


rotateTests : Test
rotateTests =
    describe "rotate behaviors"
        [ test "rotate produces negative angular acceleration" <|
            \_ ->
                let
                    config =
                        defaultConfig
                in
                rotate config
                    |> .angular
                    |> Maybe.map (expectLessThan Quantity.zero)
                    |> Maybe.withDefault (Expect.fail "Expected negative angular acceleration")
        , test "rotateCounterclockwise produces positive angular acceleration" <|
            \_ ->
                let
                    config =
                        defaultConfig
                in
                rotateCounterclockwise config
                    |> .angular
                    |> Maybe.map (expectGreaterThan Quantity.zero)
                    |> Maybe.withDefault (Expect.fail "Expected positive angular acceleration (counterclockwise spin)")
        , test "rotate behaviors produce no linear acceleration" <|
            \_ ->
                let
                    config =
                        defaultConfig
                in
                [ rotate config
                , rotateCounterclockwise config
                ]
                    |> List.map .linear
                    |> List.filterMap identity
                    |> Expect.equalLists []
        , test "rotate increases clockwise rotation over time" <|
            \_ ->
                let
                    initialKinematic =
                        atOrigin

                    trajectory =
                        simulateSteps 10 (\_ -> rotate defaultConfig) initialKinematic

                    rotations =
                        trajectory
                            |> List.map (.kinematic >> .rotation)

                    initialRotation =
                        rotations
                            |> List.head
                            |> Maybe.withDefault Quantity.zero

                    finalRotation =
                        rotations
                            |> List.reverse
                            |> List.head
                            |> Maybe.withDefault Quantity.zero
                in
                finalRotation |> expectLessThan initialRotation
        , test "rotateCounterclockwise increases counterclockwise rotation over time" <|
            \_ ->
                let
                    initialKinematic =
                        atOrigin

                    trajectory =
                        simulateSteps 10 (\_ -> rotateCounterclockwise defaultConfig) initialKinematic

                    rotations =
                        trajectory
                            |> List.map (.kinematic >> .rotation)

                    initialRotation =
                        rotations
                            |> List.head
                            |> Maybe.withDefault Quantity.zero

                    finalRotation =
                        rotations
                            |> List.reverse
                            |> List.head
                            |> Maybe.withDefault Quantity.zero
                in
                finalRotation |> expectGreaterThan initialRotation
        ]


stopRotatingTests : Test
stopRotatingTests =
    describe "stopRotating behavior"
        [ test "produces counter-rotation when spinning clockwise" <|
            \_ ->
                let
                    config =
                        defaultConfig

                    currentRotation =
                        AngularSpeed.radiansPerSecond -2
                in
                stopRotating config currentRotation
                    |> .angular
                    |> Maybe.map (expectGreaterThan Quantity.zero)
                    |> Maybe.withDefault (Expect.fail "Expected positive angular acceleration")
        , test "produces counter-rotation when spinning counterclockwise" <|
            \_ ->
                let
                    config =
                        defaultConfig

                    currentRotation =
                        AngularSpeed.radiansPerSecond 3
                in
                stopRotating config currentRotation
                    |> .angular
                    |> Maybe.map (expectLessThan Quantity.zero)
                    |> Maybe.withDefault (Expect.fail "Expected negative angular acceleration")
        , test "produces no linear acceleration" <|
            \_ ->
                let
                    config =
                        defaultConfig

                    currentRotation =
                        AngularSpeed.radiansPerSecond 2
                in
                stopRotating config currentRotation
                    |> .linear
                    |> Expect.equal Nothing
        , test "reduces rotation speed over time" <|
            \_ ->
                let
                    initialKinematic =
                        atOrigin |> withRotation (AngularSpeed.radiansPerSecond 5)

                    trajectory =
                        simulateSteps 20 (\kinematic -> stopRotating defaultConfig kinematic.rotation) initialKinematic

                    rotationSpeeds =
                        trajectory
                            |> List.map (.kinematic >> .rotation >> Quantity.abs)

                    initialRotationSpeed =
                        rotationSpeeds |> List.head |> Maybe.withDefault Quantity.zero

                    finalRotationSpeed =
                        rotationSpeeds |> List.reverse |> List.head |> Maybe.withDefault Quantity.zero
                in
                finalRotationSpeed |> expectLessThan initialRotationSpeed
        ]


lookWhereYoureGoingTests : Test
lookWhereYoureGoingTests =
    describe "lookWhereYoureGoing behavior"
        [ test "produces no steering when moving slowly" <|
            \_ ->
                let
                    source =
                        atOrigin |> withVelocity (Vector2d.metersPerSecond 0.05 0)
                in
                lookWhereYoureGoing defaultConfig source
                    |> Expect.equal none
        , test "aligns orientation with velocity direction" <|
            \_ ->
                let
                    source =
                        atOrigin
                            |> withVelocity (Vector2d.metersPerSecond 0 5)
                in
                lookWhereYoureGoing defaultConfig source
                    |> .angular
                    |> Expect.notEqual Nothing
        ]


lookWhereYoureGoingIntegrationTests : Test
lookWhereYoureGoingIntegrationTests =
    describe "lookWhereYoureGoing behavior - integration tests"
        [ test "eventually aligns orientation with velocity direction" <|
            \_ ->
                let
                    initialVelocity =
                        Vector2d.metersPerSecond 3 4

                    targetOrientation =
                        Vector2d.direction initialVelocity
                            |> Maybe.map Direction2d.toAngle
                            |> Maybe.withDefault (Angle.radians 0)

                    initialKinematic =
                        atOrigin
                            |> withVelocity initialVelocity

                    trajectory =
                        simulateSteps 30 (\kinematic -> lookWhereYoureGoing defaultConfig kinematic) initialKinematic

                    finalOrientation =
                        trajectory
                            |> List.reverse
                            |> List.head
                            |> Maybe.map (.kinematic >> .orientation)
                            |> Maybe.withDefault (Angle.radians 0)

                    angleDifference =
                        targetOrientation
                            |> Quantity.minus finalOrientation
                            |> Angle.normalize
                            |> Quantity.abs
                in
                angleDifference |> expectLessThan (Angle.radians 0.1)
        , test "consistently turns toward velocity direction over multiple steps" <|
            \_ ->
                let
                    velocity =
                        Vector2d.metersPerSecond -2 3

                    targetAngle =
                        Vector2d.direction velocity
                            |> Maybe.map Direction2d.toAngle
                            |> Maybe.withDefault (Angle.radians 0)

                    initialKinematic =
                        atOrigin
                            |> withVelocity velocity

                    trajectory =
                        simulateSteps 25 (\kinematic -> lookWhereYoureGoing defaultConfig kinematic) initialKinematic

                    angleDifferences =
                        trajectory
                            |> List.map (.kinematic >> .orientation)
                            |> List.map
                                (\angle ->
                                    targetAngle
                                        |> Quantity.minus angle
                                        |> Angle.normalize
                                        |> Quantity.abs
                                )
                            |> List.filter (Quantity.greaterThan (Angle.radians 0.1))
                in
                expectMonotonicallyDecreasing angleDifferences
        , test "does not affect velocity magnitude or add linear acceleration" <|
            \_ ->
                let
                    initialVelocity =
                        Vector2d.metersPerSecond 2 -1

                    initialKinematic =
                        atOrigin
                            |> withVelocity initialVelocity
                            |> withOrientation (Angle.radians 1.57)

                    trajectory =
                        simulateSteps 20 (\kinematic -> lookWhereYoureGoing defaultConfig kinematic) initialKinematic

                    velocityMagnitudes =
                        trajectory
                            |> List.map (.kinematic >> .velocity >> Vector2d.length)

                    initialSpeed =
                        Vector2d.length initialVelocity

                    finalSpeed =
                        velocityMagnitudes
                            |> List.reverse
                            |> List.head
                            |> Maybe.withDefault initialSpeed
                in
                Expect.all
                    [ \_ -> Expect.equal initialSpeed finalSpeed
                    , \_ ->
                        trajectory
                            |> List.map .steering
                            |> List.all (\steering -> steering.linear == Nothing)
                            |> Expect.equal True
                    ]
                    ()
        ]


lookAtTests : Test
lookAtTests =
    describe "lookAt behavior - unit tests"
        [ test "produces no angular acceleration when already facing target" <|
            \_ ->
                let
                    config =
                        defaultConfig

                    source =
                        atOrigin

                    target =
                        Point2d.meters 10 0
                in
                lookAt config source target
                    |> .angular
                    |> Expect.equal Nothing
        , test "produces no linear acceleration" <|
            \_ ->
                let
                    config =
                        defaultConfig

                    source =
                        atOrigin |> withOrientation (Angle.radians 1.57)

                    target =
                        Point2d.meters 10 0
                in
                lookAt config source target
                    |> .linear
                    |> Expect.equal Nothing
        , test "produces angular acceleration when facing away from target" <|
            \_ ->
                let
                    config =
                        defaultConfig

                    source =
                        atOrigin |> withOrientation (Angle.radians 3.14159)

                    target =
                        Point2d.meters 10 0
                in
                lookAt config source target
                    |> .angular
                    |> Expect.notEqual Nothing
        , test "produces no steering when at target position" <|
            \_ ->
                let
                    config =
                        defaultConfig

                    source =
                        atOrigin

                    target =
                        atOrigin.position
                in
                Expect.equal (lookAt config source target) none
        ]


lookAtFuzzTests : Test
lookAtFuzzTests =
    describe "lookAt behavior - fuzz tests"
        [ fuzz2 kinematicFuzzer point2dFuzzer "always respects max angular acceleration limits" <|
            \source targetPos ->
                let
                    result =
                        lookAt defaultConfig source targetPos
                in
                case result.angular of
                    Just angularAcceleration ->
                        Quantity.abs angularAcceleration |> expectLessThanOrEqualTo defaultConfig.maxAngularAcceleration

                    Nothing ->
                        if Point2d.distanceFrom source.position targetPos |> Quantity.lessThan closenessThreshold then
                            Expect.pass

                        else
                            let
                                isAlreadyAligned =
                                    case Direction2d.from source.position targetPos of
                                        Just direction ->
                                            let
                                                targetAngle =
                                                    Direction2d.toAngle direction

                                                angleDifference =
                                                    targetAngle
                                                        |> Quantity.minus source.orientation
                                                        |> Angle.normalize
                                                        |> Quantity.abs
                                            in
                                            angleDifference |> Quantity.lessThan (Angle.radians 0.02)

                                        Nothing ->
                                            True
                            in
                            if isAlreadyAligned then
                                Expect.pass

                            else
                                Expect.fail "Expected angular acceleration when not aligned"
        , fuzz kinematicFuzzer "never produces linear acceleration" <|
            \source ->
                let
                    target =
                        Point2d.meters 100 50
                in
                lookAt defaultConfig source target
                    |> .linear
                    |> Expect.equal Nothing
        , fuzz2 point2dFuzzer point2dFuzzer "produces consistent angular direction" <|
            \sourcePos targetPos ->
                if Point2d.distanceFrom sourcePos targetPos |> Quantity.greaterThan closenessThreshold then
                    let
                        targetDirection =
                            Direction2d.from sourcePos targetPos

                        misalignedOrientation =
                            case targetDirection of
                                Just direction ->
                                    Direction2d.toAngle direction |> Quantity.plus (Angle.radians 1.57)

                                Nothing ->
                                    Angle.radians 1.57

                        source =
                            atOrigin
                                |> withPosition sourcePos
                                |> withOrientation misalignedOrientation

                        result =
                            lookAt defaultConfig source targetPos
                    in
                    case ( result.angular, targetDirection ) of
                        ( Just angularAccel, Just direction ) ->
                            let
                                targetAngle =
                                    Direction2d.toAngle direction

                                currentAngle =
                                    source.orientation

                                angleDifference =
                                    targetAngle
                                        |> Quantity.minus currentAngle
                                        |> Angle.normalize

                                shouldTurnCounterclockwise =
                                    angleDifference |> Quantity.greaterThanZero

                                actuallyTurningCounterclockwise =
                                    angularAccel |> Quantity.greaterThanZero
                            in
                            if shouldTurnCounterclockwise == actuallyTurningCounterclockwise then
                                Expect.pass

                            else
                                Expect.fail "Angular acceleration direction should match required turn direction"

                        _ ->
                            Expect.pass

                else
                    Expect.pass
        ]


lookAtIntegrationTests : Test
lookAtIntegrationTests =
    describe "lookAt behavior - integration tests"
        [ test "consistently turns toward target over multiple steps" <|
            \_ ->
                let
                    target =
                        Point2d.meters 10 0

                    initialKinematic =
                        atOrigin |> withOrientation (Angle.radians 3.14159)

                    trajectory =
                        simulateSteps 30 (\kinematic -> lookAt defaultConfig kinematic target) initialKinematic

                    targetAngle =
                        Angle.radians 0

                    angleDifferences =
                        trajectory
                            |> List.map (.kinematic >> .orientation)
                            |> List.map
                                (\angle ->
                                    Quantity.abs
                                        (targetAngle
                                            |> Quantity.minus angle
                                            |> Angle.normalize
                                        )
                                )
                            |> List.filter (Quantity.greaterThan (Angle.radians 0.1))
                in
                expectMonotonicallyDecreasing angleDifferences
        , test "eventually faces target direction" <|
            \_ ->
                let
                    target =
                        Point2d.meters 5 5

                    initialKinematic =
                        atOrigin

                    trajectory =
                        simulateSteps 50 (\kinematic -> lookAt defaultConfig kinematic target) initialKinematic

                    finalOrientation =
                        trajectory
                            |> List.reverse
                            |> List.head
                            |> Maybe.map (.kinematic >> .orientation)
                            |> Maybe.withDefault (Angle.radians 0)

                    expectedAngle =
                        Angle.radians (pi / 4)

                    angleDifference =
                        Quantity.abs (expectedAngle |> Quantity.minus finalOrientation |> Angle.normalize)
                in
                angleDifference |> expectLessThan (Angle.radians 0.1)
        , test "does not affect position" <|
            \_ ->
                let
                    target =
                        Point2d.meters 10 0

                    initialKinematic =
                        atOrigin
                            |> withPosition (Point2d.meters 5 3)
                            |> withOrientation (Angle.radians 3.14159)

                    trajectory =
                        simulateSteps 20 (\kinematic -> lookAt defaultConfig kinematic target) initialKinematic

                    initialPosition =
                        initialKinematic.position

                    finalPosition =
                        trajectory
                            |> List.reverse
                            |> List.head
                            |> Maybe.map (.kinematic >> .position)
                            |> Maybe.withDefault initialPosition

                    positionDifference =
                        Point2d.distanceFrom initialPosition finalPosition
                in
                positionDifference |> expectLessThan (Length.meters 0.01)
        ]


seekTests : Test
seekTests =
    describe "seek behavior - unit tests"
        [ test "produces max acceleration toward target" <|
            \_ ->
                let
                    config =
                        defaultConfig

                    source =
                        atOrigin

                    target =
                        Point2d.meters 10 0

                    dirToTarget =
                        Direction2d.from source.position target |> Maybe.withDefault Direction2d.positiveX
                in
                seek config source target
                    |> .linear
                    |> Expect.equal (Just (maxAccelerationTowards config dirToTarget))
        , test "produces no linear acceleration when at target" <|
            \_ ->
                let
                    config =
                        defaultConfig

                    source =
                        atOrigin

                    target =
                        atOrigin.position
                in
                seek config source target
                    |> .linear
                    |> Expect.equal Nothing
        ]


seekFuzzTests : Test
seekFuzzTests =
    describe "seek behavior - fuzz tests"
        [ fuzz2 kinematicFuzzer point2dFuzzer "always respects max acceleration limits" <|
            \source targetPos ->
                let
                    result =
                        seek defaultConfig source targetPos
                in
                case result.linear of
                    Just acceleration ->
                        Vector2d.length acceleration |> expectLessThanOrEqualTo defaultConfig.maxAcceleration

                    Nothing ->
                        if Point2d.distanceFrom source.position targetPos |> Quantity.lessThan closenessThreshold then
                            Expect.pass

                        else
                            Expect.fail "Expected linear acceleration"
        ]


seekIntegrationTests : Test
seekIntegrationTests =
    describe "seek behavior - integration tests"
        [ test "consistently moves toward target over multiple steps" <|
            \_ ->
                let
                    target =
                        Point2d.meters 10 0

                    initialKinematic =
                        atOrigin |> withPosition (Point2d.meters -5 0)

                    trajectory =
                        simulateSteps 20 (\kinematic -> seek defaultConfig kinematic target) initialKinematic

                    distances =
                        List.map (distanceToTarget target) trajectory
                in
                expectMonotonicallyDecreasing distances
        , test "gets closer to target over time" <|
            \_ ->
                let
                    target =
                        Point2d.meters 5 0

                    initialKinematic =
                        atOrigin

                    initialDistance =
                        Point2d.distanceFrom initialKinematic.position target

                    trajectory =
                        simulateSteps 10 (\transform -> seek defaultConfig transform target) initialKinematic

                    finalDistance =
                        trajectory
                            |> List.reverse
                            |> List.head
                            |> Maybe.map (.kinematic >> .position >> Point2d.distanceFrom target)
                            |> Maybe.withDefault initialDistance
                in
                finalDistance |> expectLessThan initialDistance
        ]


fleeTests : Test
fleeTests =
    describe "flee behavior - unit tests"
        [ test "produces max acceleration away from target" <|
            \_ ->
                let
                    config =
                        defaultConfig

                    source =
                        atOrigin

                    target =
                        Point2d.meters 10 0

                    dirFromTarget =
                        Direction2d.from target source.position |> Maybe.withDefault Direction2d.positiveX
                in
                flee config source target
                    |> .linear
                    |> Expect.equal (Just (maxAccelerationTowards config dirFromTarget))
        ]


fleeFuzzTests : Test
fleeFuzzTests =
    describe "flee behavior - fuzz tests"
        [ fuzz2 kinematicFuzzer point2dFuzzer "always respects max acceleration limits" <|
            \source targetPos ->
                let
                    result =
                        flee defaultConfig source targetPos
                in
                case result.linear of
                    Just acceleration ->
                        Vector2d.length acceleration |> expectLessThanOrEqualTo defaultConfig.maxAcceleration

                    Nothing ->
                        Expect.fail "Expected linear acceleration"
        ]


fleeIntegrationTests : Test
fleeIntegrationTests =
    describe "flee behavior - integration tests"
        [ test "consistently moves away from target over multiple steps" <|
            \_ ->
                let
                    target =
                        Point2d.meters 10 0

                    initialKinematic =
                        atOrigin |> withPosition (Point2d.meters -5 0)

                    trajectory =
                        simulateSteps 20 (\kinematic -> flee defaultConfig kinematic target) initialKinematic

                    distances =
                        List.map (distanceToTarget target) trajectory
                in
                expectMonotonicallyIncreasing distances
        , test "gets further away from target over time" <|
            \_ ->
                let
                    target =
                        Point2d.meters 5 0

                    initialKinematic =
                        atOrigin

                    initialDistance =
                        Point2d.distanceFrom initialKinematic.position target

                    trajectory =
                        simulateSteps 10 (\transform -> flee defaultConfig transform target) initialKinematic

                    finalDistance =
                        trajectory
                            |> List.reverse
                            |> List.head
                            |> Maybe.map (.kinematic >> .position >> Point2d.distanceFrom target)
                            |> Maybe.withDefault initialDistance
                in
                finalDistance |> expectGreaterThan initialDistance
        ]


arriveTests : Test
arriveTests =
    describe "arrive behavior - unit tests"
        [ test "produces deceleration when close to target" <|
            \_ ->
                let
                    config =
                        defaultConfig

                    source =
                        atOrigin
                            |> withPosition (Point2d.meters 4 0)
                            |> withVelocity (Vector2d.withLength (Speed.metersPerSecond 5) Direction2d.positiveX)

                    target =
                        Point2d.meters 5 0
                in
                case arrive config source target |> .linear of
                    Just acceleration ->
                        Vector2d.dot acceleration source.velocity
                            |> expectLessThan Quantity.zero

                    _ ->
                        Expect.fail "Expected linear steering"
        , test "produces no linear acceleration when at target" <|
            \_ ->
                let
                    config =
                        defaultConfig

                    source =
                        atOrigin

                    target =
                        atOrigin.position
                in
                arrive config source target
                    |> .linear
                    |> Expect.equal Nothing
        , test "produces less acceleration than seek when far from target" <|
            \_ ->
                let
                    config =
                        defaultConfig

                    source =
                        atOrigin
                            |> withPosition (Point2d.meters 0 0)

                    target =
                        Point2d.meters 20 0

                    arriveResult =
                        arrive config source target

                    seekResult =
                        seek config source target
                in
                case ( arriveResult.linear, seekResult.linear ) of
                    ( Just arriveAcceleration, Just seekAcceleration ) ->
                        Vector2d.length arriveAcceleration |> expectLessThanOrEqualTo (Vector2d.length seekAcceleration)

                    _ ->
                        Expect.fail "Both behaviors should produce acceleration"
        ]


arriveFuzzTests : Test
arriveFuzzTests =
    describe "arrive behavior - fuzz tests"
        [ fuzz2 kinematicFuzzer point2dFuzzer "always respects max acceleration limits" <|
            \source targetPos ->
                let
                    result =
                        arrive defaultConfig source targetPos
                in
                case result.linear of
                    Just acceleration ->
                        Vector2d.length acceleration |> expectLessThanOrEqualTo defaultConfig.maxAcceleration

                    Nothing ->
                        if Point2d.distanceFrom source.position targetPos |> Quantity.lessThan closenessThreshold then
                            Expect.pass

                        else
                            Expect.fail "Expected linear acceleration"
        , fuzz kinematicFuzzer "produces deceleration when moving fast toward close target" <|
            \source ->
                let
                    fastVelocity =
                        Vector2d.metersPerSecond 8 0

                    fastSource =
                        { source | velocity = fastVelocity }

                    closeTarget =
                        fastSource.position
                            |> Point2d.translateBy (Vector2d.meters 1 0)
                in
                case arrive defaultConfig fastSource closeTarget |> .linear of
                    Just acceleration ->
                        Vector2d.dot acceleration fastSource.velocity
                            |> expectLessThan Quantity.zero

                    _ ->
                        Expect.fail "Expected linear steering"
        ]


arriveIntegrationTests : Test
arriveIntegrationTests =
    describe "arrive behavior - integration tests"
        [ test "slows down as it approaches target" <|
            \_ ->
                let
                    target =
                        Point2d.meters 10 0

                    slowingRadius =
                        Length.meters 5.0

                    initialKinematic =
                        atOrigin

                    trajectory =
                        simulateSteps 100 (\transform -> arrive defaultConfig transform target) initialKinematic

                    measurements =
                        List.map .kinematic trajectory

                    indexedVelocities =
                        List.indexedMap Tuple.pair measurements

                    slowdownStartIndex =
                        indexedVelocities
                            |> List.foldl
                                (\( index, transform ) result ->
                                    case result of
                                        Just _ ->
                                            result

                                        Nothing ->
                                            if
                                                Point2d.distanceFrom transform.position target
                                                    |> Quantity.lessThan slowingRadius
                                            then
                                                Just index

                                            else
                                                result
                                )
                                Nothing
                            |> Maybe.withDefault 0

                    decelerationPhaseSlice =
                        measurements
                            |> List.drop (slowdownStartIndex + 1)
                            |> List.map (.velocity >> Vector2d.length)
                            |> List.filter (Quantity.greaterThan (Speed.metersPerSecond 0.5))
                in
                if List.length decelerationPhaseSlice >= 5 then
                    expectMonotonicallyDecreasing decelerationPhaseSlice

                else
                    Expect.fail "Expected several measurements for deceleration"
        , test "eventually stops at target" <|
            \_ ->
                let
                    target =
                        Point2d.meters 5 0

                    initialKinematic =
                        atOrigin

                    finalDistance =
                        simulateUntilConverged (\transform targetPos -> arrive defaultConfig transform targetPos) target initialKinematic
                in
                finalDistance |> expectLessThan (Length.meters 0.5)
        , test "reaches target without excessive oscillation" <|
            \_ ->
                let
                    target =
                        Point2d.meters 8 0

                    initialKinematic =
                        atOrigin

                    finalDistance =
                        simulateUntilConverged
                            (\transform targetPos -> arrive defaultConfig transform targetPos)
                            target
                            initialKinematic
                in
                finalDistance |> expectLessThan (Length.meters 0.3)
        ]


wanderTests : Test
wanderTests =
    describe "wander behavior - unit tests"
        [ test "produces linear acceleration" <|
            \_ ->
                let
                    config =
                        defaultConfig

                    wanderConfig =
                        defaultWanderConfig

                    wanderAngle =
                        Angle.radians 0

                    source =
                        atOrigin
                in
                wander config wanderConfig defaultSeed wanderAngle source
                    |> (\( steering, _, _ ) -> steering.linear)
                    |> Expect.notEqual Nothing
        , test "produces angular acceleration when not aligned with target" <|
            \_ ->
                let
                    config =
                        defaultConfig

                    wanderConfig =
                        defaultWanderConfig

                    wanderAngle =
                        Angle.radians 1.57

                    source =
                        atOrigin
                in
                wander config wanderConfig defaultSeed wanderAngle source
                    |> (\( steering, _, _ ) -> steering.angular)
                    |> Expect.notEqual Nothing
        , test "updates wander angle" <|
            \_ ->
                let
                    config =
                        defaultConfig

                    wanderConfig =
                        defaultWanderConfig

                    initialWanderAngle =
                        Angle.radians 0

                    source =
                        atOrigin

                    ( _, newWanderAngle, _ ) =
                        wander config wanderConfig defaultSeed initialWanderAngle source
                in
                newWanderAngle |> Expect.notEqual initialWanderAngle
        , test "updates random seed" <|
            \_ ->
                let
                    config =
                        defaultConfig

                    wanderConfig =
                        defaultWanderConfig

                    wanderAngle =
                        Angle.radians 0

                    source =
                        atOrigin

                    initialSeed =
                        defaultSeed

                    ( _, _, newSeed ) =
                        wander config wanderConfig initialSeed wanderAngle source
                in
                newSeed |> Expect.notEqual initialSeed
        ]


wanderFuzzTests : Test
wanderFuzzTests =
    describe "wander behavior - fuzz tests"
        [ fuzz3 kinematicFuzzer angleFuzzer seedFuzzer "always respects max acceleration limits" <|
            \source wanderAngle seed ->
                let
                    ( steering, _, _ ) =
                        wander defaultConfig defaultWanderConfig seed wanderAngle source
                in
                case steering.linear of
                    Just acceleration ->
                        Vector2d.length acceleration |> expectLessThanOrEqualTo defaultConfig.maxAcceleration

                    Nothing ->
                        Expect.fail "Wander should always produce linear acceleration"
        , fuzz3 kinematicFuzzer angleFuzzer seedFuzzer "respects max angular acceleration limits" <|
            \source wanderAngle seed ->
                let
                    ( steering, _, _ ) =
                        wander defaultConfig defaultWanderConfig seed wanderAngle source
                in
                case steering.angular of
                    Just angularAcceleration ->
                        Quantity.abs angularAcceleration |> expectLessThanOrEqualTo defaultConfig.maxAngularAcceleration

                    Nothing ->
                        Expect.pass
        , fuzz3 kinematicFuzzer angleFuzzer seedFuzzer "produces deterministic results for same inputs" <|
            \source wanderAngle seed ->
                let
                    ( steering1, angle1, seed1 ) =
                        wander defaultConfig defaultWanderConfig seed wanderAngle source

                    ( steering2, angle2, seed2 ) =
                        wander defaultConfig defaultWanderConfig seed wanderAngle source
                in
                Expect.all
                    [ \_ -> steering1 |> Expect.equal steering2
                    , \_ -> angle1 |> Expect.equal angle2
                    , \_ -> seed1 |> Expect.equal seed2
                    ]
                    ()
        ]


wanderIntegrationTests : Test
wanderIntegrationTests =
    let
        simulateWanderSteps numSteps initialSeed steeringConfig initialWanderAngle initialKinematic =
            let
                wanderBehavior transform ( seed, wanderAngle ) =
                    let
                        ( steering, nextWanderAngle, nextSeed ) =
                            wander steeringConfig defaultWanderConfig seed wanderAngle transform
                    in
                    ( steering, ( nextSeed, nextWanderAngle ) )

                results =
                    simulateStepsWithState numSteps wanderBehavior ( initialSeed, initialWanderAngle ) initialKinematic
            in
            List.map
                (\{ step, kinematic, steering, state } ->
                    let
                        ( seed, wanderAngle ) =
                            state
                    in
                    { step = step
                    , kinematic = kinematic
                    , steering = steering
                    , wanderAngle = wanderAngle
                    , seed = seed
                    }
                )
                results
    in
    describe "wander behavior - integration tests"
        [ test "creates curved movement paths" <|
            \_ ->
                let
                    initialKinematic =
                        atOrigin

                    initialWanderAngle =
                        Angle.radians 0

                    trajectory =
                        simulateWanderSteps 30 defaultSeed defaultConfig initialWanderAngle initialKinematic

                    positions =
                        trajectory |> List.map (.kinematic >> .position)

                    directions =
                        positions
                            |> List.map2 Vector2d.from (List.drop 1 positions)
                            |> List.map Vector2d.direction
                            |> List.filterMap identity
                            |> List.map Direction2d.toAngle

                    hasVariation =
                        case directions of
                            first :: rest ->
                                rest
                                    |> List.any
                                        (\angle ->
                                            angle
                                                |> Quantity.minus first
                                                |> Angle.normalize
                                                |> Quantity.abs
                                                |> Quantity.greaterThan (Angle.radians 0.1)
                                        )

                            [] ->
                                False
                in
                if hasVariation then
                    Expect.pass

                else
                    Expect.fail "Wander should create curved paths, not straight lines"
        , test "wander angle evolves over time" <|
            \_ ->
                let
                    initialKinematic =
                        atOrigin

                    seed =
                        Random.initialSeed 54321

                    initialWanderAngle =
                        Angle.radians 0

                    trajectory =
                        simulateWanderSteps 20 seed defaultConfig initialWanderAngle initialKinematic

                    wanderAngles =
                        trajectory |> List.map .wanderAngle

                    finalWanderAngle =
                        wanderAngles
                            |> List.reverse
                            |> List.head
                            |> Maybe.withDefault initialWanderAngle

                    totalChange =
                        finalWanderAngle
                            |> Quantity.minus initialWanderAngle
                            |> Quantity.abs
                in
                totalChange |> expectGreaterThan (Angle.radians 0.05)
        , test "agent keeps moving forward" <|
            \_ ->
                let
                    initialKinematic =
                        atOrigin

                    seed =
                        Random.initialSeed 98765

                    initialWanderAngle =
                        Angle.radians 0

                    trajectory =
                        simulateWanderSteps 25 seed defaultConfig initialWanderAngle initialKinematic

                    positions =
                        trajectory |> List.map (.kinematic >> .position)

                    initialPosition =
                        positions
                            |> List.head
                            |> Maybe.withDefault Point2d.origin

                    finalPosition =
                        positions
                            |> List.reverse
                            |> List.head
                            |> Maybe.withDefault Point2d.origin

                    totalDistance =
                        Point2d.distanceFrom initialPosition finalPosition
                in
                totalDistance |> expectGreaterThan (Length.meters 1.0)
        ]


wallAvoidanceTests : Test
wallAvoidanceTests =
    describe "wallAvoidance behavior - unit tests"
        [ test "produces no steering when no collision detected" <|
            \_ ->
                let
                    config =
                        defaultConfig

                    source =
                        atOrigin

                    noCollisionDetector _ _ _ =
                        Nothing
                in
                wallAvoidance config noCollisionDetector (Length.meters 5) source
                    |> Expect.equal none
        , test "produces linear steering when collision detected" <|
            \_ ->
                let
                    config =
                        defaultConfig

                    source =
                        atOrigin |> withVelocity (Vector2d.metersPerSecond 3 0)

                    collisionDetector _ _ _ =
                        Just ( Point2d.meters 2 0, Direction2d.negativeX )
                in
                wallAvoidance config collisionDetector (Length.meters 5) source
                    |> .linear
                    |> Expect.notEqual Nothing
        , test "produces no angular steering" <|
            \_ ->
                let
                    config =
                        defaultConfig

                    source =
                        atOrigin |> withVelocity (Vector2d.metersPerSecond 3 0)

                    collisionDetector _ _ _ =
                        Just ( Point2d.meters 2 0, Direction2d.negativeX )
                in
                wallAvoidance config collisionDetector (Length.meters 5) source
                    |> .angular
                    |> Expect.equal Nothing
        , test "steers perpendicular to wall normal + breaking velocity offset" <|
            \_ ->
                let
                    config =
                        defaultConfig

                    source =
                        atOrigin |> withVelocity (Vector2d.metersPerSecond 3 0)

                    wallNormal =
                        Direction2d.positiveY

                    collisionDetector _ _ _ =
                        Just ( Point2d.meters 2 0, wallNormal )
                in
                case wallAvoidance config collisionDetector (Length.meters 5) source |> .linear of
                    Just steeringForce ->
                        let
                            expectedDirection =
                                -- The non-standard breaking component of the behavior distorts the expected angle
                                Direction2d.perpendicularTo wallNormal |> Direction2d.rotateBy (Angle.degrees -45)

                            actualDirection =
                                Vector2d.direction steeringForce
                        in
                        case ( expectedDirection, actualDirection ) of
                            ( expected, Just actual ) ->
                                let
                                    angleDiff =
                                        Direction2d.angleFrom expected actual |> Quantity.abs
                                in
                                angleDiff |> expectLessThan (Angle.degrees 5)

                            _ ->
                                Expect.fail "Expected valid directions"

                    _ ->
                        Expect.fail "Expected linear steering"
        , test "stronger avoidance when closer to wall" <|
            \_ ->
                let
                    config =
                        defaultConfig

                    source =
                        atOrigin |> withVelocity (Vector2d.metersPerSecond 3 0)

                    probeDistance =
                        Length.meters 5

                    closeCollisionDetector _ _ _ =
                        Just ( Point2d.meters 1 0, Direction2d.negativeX )

                    farCollisionDetector _ _ _ =
                        Just ( Point2d.meters 4 0, Direction2d.negativeX )

                    closeForce =
                        wallAvoidance config closeCollisionDetector probeDistance source
                            |> .linear
                            |> Maybe.map Vector2d.length
                            |> Maybe.withDefault Quantity.zero

                    farForce =
                        wallAvoidance config farCollisionDetector probeDistance source
                            |> .linear
                            |> Maybe.map Vector2d.length
                            |> Maybe.withDefault Quantity.zero
                in
                closeForce |> expectGreaterThan farForce
        ]


wallAvoidanceFuzzTests : Test
wallAvoidanceFuzzTests =
    describe "wallAvoidance behavior - fuzz tests"
        [ fuzz2 kinematicFuzzer (Fuzz.floatRange 1 10) "always respects max acceleration limits" <|
            \source probeDistanceFloat ->
                let
                    probeDistance =
                        Length.meters probeDistanceFloat

                    collisionDetector _ _ _ =
                        Just ( Point2d.meters 2 0, Direction2d.positiveY )

                    result =
                        wallAvoidance defaultConfig collisionDetector probeDistance source
                in
                case result.linear of
                    Just acceleration ->
                        Vector2d.length acceleration |> expectLessThanOrEqualTo defaultConfig.maxAcceleration

                    Nothing ->
                        Expect.pass
        , fuzz kinematicFuzzer "never produces angular steering" <|
            \source ->
                let
                    collisionDetector _ _ _ =
                        Just ( Point2d.meters 5 5, Direction2d.negativeX )
                in
                wallAvoidance defaultConfig collisionDetector (Length.meters 5) source
                    |> .angular
                    |> Expect.equal Nothing
        ]


wallAvoidanceIntegrationTests : Test
wallAvoidanceIntegrationTests =
    describe "wallAvoidance behavior - integration tests"
        [ test "successfully avoids head-on collision with wall" <|
            \_ ->
                let
                    initialKinematic =
                        atOrigin
                            |> withPosition (Point2d.meters -5 0)
                            |> withVelocity (Vector2d.metersPerSecond 2 0)

                    wallCollisionDetector startPoint direction probeDistance =
                        let
                            endPoint =
                                startPoint |> Point2d.translateIn direction probeDistance

                            ray =
                                LineSegment2d.from startPoint endPoint

                            wallSegment =
                                LineSegment2d.from (Point2d.meters 0 -10) (Point2d.meters 0 10)
                        in
                        LineSegment2d.intersectionPoint wallSegment ray
                            |> Maybe.map (\point -> ( point, Direction2d.negativeX ))

                    trajectory =
                        simulateSteps 30
                            (\kinematic -> wallAvoidance defaultConfig wallCollisionDetector (Length.meters 3) kinematic)
                            initialKinematic

                    finalPosition =
                        trajectory
                            |> List.reverse
                            |> List.head
                            |> Maybe.map (.kinematic >> .position)
                            |> Maybe.withDefault initialKinematic.position

                    finalX =
                        Point2d.xCoordinate finalPosition
                in
                finalX |> expectLessThan Quantity.zero
        , test "maintains forward progress while avoiding wall" <|
            \_ ->
                let
                    initialKinematic =
                        atOrigin
                            |> withPosition (Point2d.meters -8 -2)
                            |> withVelocity (Vector2d.metersPerSecond 3 1)

                    wallCollisionDetector startPoint direction probeDistance =
                        let
                            endPoint =
                                startPoint |> Point2d.translateIn direction probeDistance

                            ray =
                                LineSegment2d.from startPoint endPoint

                            wallSegment =
                                LineSegment2d.from (Point2d.meters -20 0) (Point2d.meters 20 0)
                        in
                        LineSegment2d.intersectionPoint wallSegment ray
                            |> Maybe.map (\point -> ( point, Direction2d.negativeY ))

                    trajectory =
                        simulateSteps 25
                            (\kinematic -> wallAvoidance defaultConfig wallCollisionDetector (Length.meters 4) kinematic)
                            initialKinematic

                    positions =
                        trajectory |> List.map (.kinematic >> .position)

                    initialX =
                        Point2d.xCoordinate initialKinematic.position

                    finalX =
                        positions
                            |> List.reverse
                            |> List.head
                            |> Maybe.map Point2d.xCoordinate
                            |> Maybe.withDefault initialX
                in
                finalX |> expectGreaterThan initialX
        , test "does not oscillate when running parallel to wall" <|
            \_ ->
                let
                    initialKinematic =
                        atOrigin
                            |> withPosition (Point2d.meters -2 -1)
                            |> withVelocity (Vector2d.metersPerSecond 0 3)

                    wallCollisionDetector startPoint direction probeDistance =
                        let
                            endPoint =
                                startPoint |> Point2d.translateIn direction probeDistance

                            ray =
                                LineSegment2d.from startPoint endPoint

                            wallSegment =
                                LineSegment2d.from (Point2d.meters 0 -10) (Point2d.meters 0 10)
                        in
                        LineSegment2d.intersectionPoint wallSegment ray
                            |> Maybe.map (\point -> ( point, Direction2d.negativeX ))

                    trajectory =
                        simulateSteps 20
                            (\kinematic -> wallAvoidance defaultConfig wallCollisionDetector (Length.meters 3) kinematic)
                            initialKinematic

                    xPositions =
                        trajectory
                            |> List.map (.kinematic >> .position >> Point2d.xCoordinate >> Length.inMeters)

                    avgX =
                        List.sum xPositions / toFloat (List.length xPositions)

                    variance =
                        if List.length xPositions > 0 then
                            xPositions
                                |> List.map (\x -> (x - avgX) ^ 2)
                                |> List.sum
                                |> (\sum -> sum / toFloat (List.length xPositions))

                        else
                            0
                in
                variance |> Expect.lessThan 0.1
        ]



---
--- Test helpers
---


expectGreaterThan : Quantity number units -> Quantity number units -> Expect.Expectation
expectGreaterThan baseline value =
    if value |> Quantity.greaterThan baseline then
        Expect.pass

    else
        Expect.fail
            (String.join " "
                [ "Value not greater than baseline"
                , "expected:"
                , Debug.toString baseline
                , "actual:"
                , Debug.toString value
                ]
            )


expectLessThan : Quantity number units -> Quantity number units -> Expect.Expectation
expectLessThan baseline value =
    if value |> Quantity.lessThan baseline then
        Expect.pass

    else
        Expect.fail
            (String.join " "
                [ "Value not less than baseline."
                , "expected:"
                , Debug.toString baseline
                , "actual:"
                , Debug.toString value
                ]
            )


expectLessThanOrEqualTo : Quantity Float units -> Quantity Float units -> Expect.Expectation
expectLessThanOrEqualTo baseline value =
    let
        tolerance =
            Quantity.multiplyBy 1.0e-10 baseline
    in
    if value |> Quantity.lessThanOrEqualTo (baseline |> Quantity.plus tolerance) then
        Expect.pass

    else
        Expect.fail
            (String.join " "
                [ "Value not less than or equal to baseline"
                , "expected:"
                , Debug.toString baseline
                , "actual:"
                , Debug.toString value
                ]
            )


expectMonotonicallyIncreasing : List (Quantity number units) -> Expect.Expectation
expectMonotonicallyIncreasing values =
    case values of
        [] ->
            Expect.pass

        [ _ ] ->
            Expect.pass

        first :: second :: rest ->
            if second |> Quantity.greaterThanOrEqualTo first then
                expectMonotonicallyIncreasing (second :: rest)

            else
                Expect.fail "Expected monotonically increasing values"


expectMonotonicallyDecreasing : List (Quantity number units) -> Expect.Expectation
expectMonotonicallyDecreasing values =
    case values of
        [] ->
            Expect.pass

        [ _ ] ->
            Expect.pass

        first :: second :: rest ->
            if first |> Quantity.greaterThanOrEqualTo second then
                expectMonotonicallyDecreasing (second :: rest)

            else
                Expect.fail "Expected monotonically decreasing values"


applySteering :
    SteeringConfig2d
    -> Duration
    -> Steering2d coords
    -> Kinematic2d coords
    -> Kinematic2d coords
applySteering steeringConfig delta steering kinematic =
    let
        nextVelocity =
            case steering.linear of
                Just accelerationVector ->
                    kinematic.velocity
                        |> Vector2d.plus (Vector2d.for delta accelerationVector)

                Nothing ->
                    kinematic.velocity

        clampedVelocity =
            let
                currentSpeed =
                    Vector2d.length nextVelocity
            in
            if currentSpeed |> Quantity.greaterThan steeringConfig.maxVelocity then
                Vector2d.scaleTo steeringConfig.maxVelocity nextVelocity

            else
                nextVelocity

        nextPosition =
            kinematic.position
                |> Point2d.translateBy (Vector2d.for delta clampedVelocity)

        nextRotation =
            case steering.angular of
                Just angularAcceleration ->
                    kinematic.rotation
                        |> Quantity.plus (Quantity.for delta angularAcceleration)
                        |> Quantity.clamp
                            (Quantity.negate steeringConfig.maxRotation)
                            steeringConfig.maxRotation

                Nothing ->
                    kinematic.rotation

        nextOrientation =
            kinematic.orientation
                |> Quantity.plus (Quantity.for delta nextRotation)
                |> Angle.normalize
    in
    { position = nextPosition
    , velocity = clampedVelocity
    , orientation = nextOrientation
    , rotation = nextRotation
    }


maxAccelerationTowards :
    SteeringConfig2d
    -> Direction2d coords
    -> Vector2d Acceleration.MetersPerSecondSquared coords
maxAccelerationTowards steeringConfig dir =
    Vector2d.withLength steeringConfig.maxAcceleration dir



-- Integration


type alias SimulationStep state =
    { step : Int
    , kinematic : Kinematic2d TestCoordinates
    , steering : Steering2d TestCoordinates
    , state : state
    }


simulateSteps :
    Int
    -> (Kinematic2d TestCoordinates -> Steering2d TestCoordinates)
    -> Kinematic2d TestCoordinates
    -> List (SimulationStep ())
simulateSteps numSteps behaviorFn initialKinematic =
    simulateStepsWithState numSteps (\kinematic _ -> ( behaviorFn kinematic, () )) () initialKinematic


simulateStepsWithState :
    Int
    -> (Kinematic2d TestCoordinates -> state -> ( Steering2d TestCoordinates, state ))
    -> state
    -> Kinematic2d TestCoordinates
    -> List (SimulationStep state)
simulateStepsWithState numSteps behaviorFn initialState initialKinematic =
    simulateStepsHelper numSteps behaviorFn initialState initialKinematic 0 []


simulateStepsHelper :
    Int
    -> (Kinematic2d TestCoordinates -> state -> ( Steering2d TestCoordinates, state ))
    -> state
    -> Kinematic2d TestCoordinates
    -> Int
    -> List (SimulationStep state)
    -> List (SimulationStep state)
simulateStepsHelper remaining behaviorFn currentState currentKinematic stepNum acc =
    if remaining <= 0 then
        List.reverse acc

    else
        let
            ( steering, nextState ) =
                behaviorFn currentKinematic currentState

            nextTransform =
                applySteering defaultConfig (Duration.seconds 0.1) steering currentKinematic

            step =
                { step = stepNum
                , kinematic = currentKinematic
                , steering = steering
                , state = currentState
                }
        in
        simulateStepsHelper (remaining - 1) behaviorFn nextState nextTransform (stepNum + 1) (step :: acc)


distanceToTarget : Point2d Length.Meters TestCoordinates -> SimulationStep state -> Length.Length
distanceToTarget target step =
    Point2d.distanceFrom step.kinematic.position target


type alias ConvergenceConfig =
    { maxIterations : Int
    , convergenceThreshold : Length.Length
    , progressThreshold : Length.Length
    , timeStep : Duration
    }


defaultConvergenceConfig : ConvergenceConfig
defaultConvergenceConfig =
    { maxIterations = 1000
    , convergenceThreshold = Length.meters 0.1
    , progressThreshold = Length.meters 0.001
    , timeStep = Duration.seconds 0.1
    }


initialLastDistance : Length.Length
initialLastDistance =
    Length.meters 100


simulateUntilConverged :
    (Kinematic2d TestCoordinates -> Point2d Length.Meters TestCoordinates -> Steering2d TestCoordinates)
    -> Point2d Length.Meters TestCoordinates
    -> Kinematic2d TestCoordinates
    -> Length.Length
simulateUntilConverged behaviorFn target initialKinematic =
    simulateUntilConvergedWith defaultConvergenceConfig behaviorFn target initialKinematic


simulateUntilConvergedWith :
    ConvergenceConfig
    -> (Kinematic2d TestCoordinates -> Point2d Length.Meters TestCoordinates -> Steering2d TestCoordinates)
    -> Point2d Length.Meters TestCoordinates
    -> Kinematic2d TestCoordinates
    -> Length.Length
simulateUntilConvergedWith config behaviorFn target initialKinematic =
    simulateUntilConvergedHelper config behaviorFn target initialKinematic 0 initialLastDistance


simulateUntilConvergedHelper :
    ConvergenceConfig
    -> (Kinematic2d TestCoordinates -> Point2d Length.Meters TestCoordinates -> Steering2d TestCoordinates)
    -> Point2d Length.Meters TestCoordinates
    -> Kinematic2d TestCoordinates
    -> Int
    -> Length.Length
    -> Length.Length
simulateUntilConvergedHelper config behaviorFn target currentTransform iterations lastDistance =
    if iterations > config.maxIterations then
        lastDistance

    else
        let
            currentDistance =
                Point2d.distanceFrom currentTransform.position target

            steering =
                behaviorFn currentTransform target

            nextTransform =
                applySteering defaultConfig config.timeStep steering currentTransform
        in
        if currentDistance |> Quantity.lessThan config.convergenceThreshold then
            currentDistance

        else if
            Quantity.abs
                (currentDistance |> Quantity.minus lastDistance)
                |> Quantity.lessThan config.progressThreshold
        then
            currentDistance

        else
            simulateUntilConvergedHelper config behaviorFn target nextTransform (iterations + 1) currentDistance
