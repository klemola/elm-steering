module Steering2dTests exposing (suite)

import Acceleration
import Angle exposing (Angle)
import AngularAcceleration
import AngularSpeed exposing (AngularSpeed)
import Direction2d
import Duration exposing (Duration)
import Expect
import Fuzz exposing (Fuzzer)
import Length
import Point2d exposing (Point2d)
import Quantity exposing (Quantity)
import Random
import Set
import Speed exposing (Speed)
import Steering2d
    exposing
        ( Steering2d
        , SteeringConfig2d
        , Transform2d
        , WanderConfig2d
        , accelerate
        , arrive
        , decelerate
        , lookAt
        , none
        , rotate
        , rotateCounterclockwise
        , seek
        , stopRotating
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
import Vector2d


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


atOrigin : Transform2d TestCoordinates
atOrigin =
    { position = Point2d.origin
    , orientation = Quantity.zero
    , velocity = Quantity.zero
    , rotation = Quantity.zero
    }


withPosition : Point2d Length.Meters TestCoordinates -> Transform2d TestCoordinates -> Transform2d TestCoordinates
withPosition position transform =
    { transform | position = position }


withOrientation : Angle -> Transform2d TestCoordinates -> Transform2d TestCoordinates
withOrientation orientation transform =
    { transform | orientation = orientation }


withVelocity : Speed -> Transform2d TestCoordinates -> Transform2d TestCoordinates
withVelocity velocity transform =
    { transform | velocity = velocity }


withRotation : AngularSpeed -> Transform2d TestCoordinates -> Transform2d TestCoordinates
withRotation rotation transform =
    { transform | rotation = rotation }


point2dFuzzer : Fuzzer (Point2d Length.Meters TestCoordinates)
point2dFuzzer =
    Fuzz.map2 Point2d.meters
        (Fuzz.floatRange -100 100)
        (Fuzz.floatRange -100 100)


transformFuzzer : Fuzzer (Transform2d TestCoordinates)
transformFuzzer =
    Fuzz.map4
        Transform2d
        point2dFuzzer
        (Fuzz.map Angle.radians (Fuzz.floatRange -3.14 3.14))
        (Fuzz.map Speed.metersPerSecond (Fuzz.floatRange -10 10))
        (Fuzz.map AngularSpeed.radiansPerSecond (Fuzz.floatRange -5 5))


angleFuzzer : Fuzzer Angle
angleFuzzer =
    Fuzz.map Angle.radians (Fuzz.floatRange -6.28 6.28)


seedFuzzer : Fuzzer Random.Seed
seedFuzzer =
    Fuzz.map Random.initialSeed (Fuzz.intRange 0 1000000)


suite : Test
suite =
    describe "Steering2d behaviors"
        [ basicsSmokeTests
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
                accelerate config
                    |> .linear
                    |> Expect.equal (Just config.maxAcceleration)
        , test "produces no angular acceleration" <|
            \_ ->
                let
                    config =
                        defaultConfig
                in
                accelerate config
                    |> .angular
                    |> Expect.equal Nothing
        , test "increases velocity over time" <|
            \_ ->
                let
                    initialTransform =
                        atOrigin |> withVelocity (Speed.metersPerSecond 2)

                    trajectory =
                        simulateSteps 10 (\_ -> accelerate defaultConfig) initialTransform

                    velocities =
                        trajectory
                            |> List.map (.transform >> .velocity >> Quantity.abs)

                    initialVelocity =
                        velocities |> List.head |> Maybe.withDefault Quantity.zero

                    finalVelocity =
                        velocities |> List.reverse |> List.head |> Maybe.withDefault Quantity.zero
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

                    currentVelocity =
                        Speed.metersPerSecond 5
                in
                decelerate config currentVelocity
                    |> .linear
                    |> Maybe.map (expectLessThan Quantity.zero)
                    |> Maybe.withDefault (Expect.fail "Expected negative acceleration")
        , test "produces acceleration when moving backward" <|
            \_ ->
                let
                    config =
                        defaultConfig

                    currentVelocity =
                        Speed.metersPerSecond -3
                in
                decelerate config currentVelocity
                    |> .linear
                    |> Maybe.map (expectGreaterThan Quantity.zero)
                    |> Maybe.withDefault (Expect.fail "Expected positive acceleration")
        , test "produces no angular acceleration" <|
            \_ ->
                let
                    config =
                        defaultConfig

                    currentVelocity =
                        Speed.metersPerSecond 5
                in
                decelerate config currentVelocity
                    |> .angular
                    |> Expect.equal Nothing
        , test "reduces speed over time" <|
            \_ ->
                let
                    initialTransform =
                        atOrigin |> withVelocity (Speed.metersPerSecond 8)

                    trajectory =
                        simulateSteps 15 (\transform -> decelerate defaultConfig transform.velocity) initialTransform

                    speeds =
                        trajectory
                            |> List.map (.transform >> .velocity >> Quantity.abs)

                    initialSpeed =
                        speeds |> List.head |> Maybe.withDefault Quantity.zero

                    finalSpeed =
                        speeds |> List.reverse |> List.head |> Maybe.withDefault Quantity.zero
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
                    -- Expected result: bunch of Nothings are cleaned to produce an empty list
                    |> Expect.equalLists []
        , test "rotate increases clockwise rotation over time" <|
            \_ ->
                let
                    initialTransform =
                        atOrigin

                    trajectory =
                        simulateSteps 10 (\_ -> rotate defaultConfig) initialTransform

                    rotations =
                        trajectory
                            |> List.map (.transform >> .rotation)

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
                    initialTransform =
                        atOrigin

                    trajectory =
                        simulateSteps 10 (\_ -> rotateCounterclockwise defaultConfig) initialTransform

                    rotations =
                        trajectory
                            |> List.map (.transform >> .rotation)

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
                    initialTransform =
                        atOrigin |> withRotation (AngularSpeed.radiansPerSecond 5)

                    trajectory =
                        simulateSteps 20 (\transform -> stopRotating defaultConfig transform.rotation) initialTransform

                    rotationSpeeds =
                        trajectory
                            |> List.map (.transform >> .rotation >> Quantity.abs)

                    initialRotationSpeed =
                        rotationSpeeds |> List.head |> Maybe.withDefault Quantity.zero

                    finalRotationSpeed =
                        rotationSpeeds |> List.reverse |> List.head |> Maybe.withDefault Quantity.zero
                in
                finalRotationSpeed |> expectLessThan initialRotationSpeed
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
        [ fuzz2 transformFuzzer point2dFuzzer "always respects max angular acceleration limits" <|
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
        , fuzz transformFuzzer "never produces linear acceleration" <|
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

                    initialTransform =
                        atOrigin |> withOrientation (Angle.radians 3.14159)

                    trajectory =
                        simulateSteps 30 (\transform -> lookAt defaultConfig transform target) initialTransform

                    targetAngle =
                        Angle.radians 0

                    angleDifferences =
                        trajectory
                            |> List.map (.transform >> .orientation)
                            |> List.map
                                (\angle ->
                                    Quantity.abs
                                        (targetAngle
                                            |> Quantity.minus angle
                                            |> Angle.normalize
                                        )
                                )
                            -- Stop measuring before expected oscillation around the target orientation begins
                            |> List.filter (Quantity.greaterThan (Angle.radians 0.1))
                in
                expectMonotonicallyDecreasing angleDifferences
        , test "eventually faces target direction" <|
            \_ ->
                let
                    target =
                        Point2d.meters 5 5

                    initialTransform =
                        atOrigin

                    trajectory =
                        simulateSteps 50 (\transform -> lookAt defaultConfig transform target) initialTransform

                    finalOrientation =
                        trajectory
                            |> List.reverse
                            |> List.head
                            |> Maybe.map (.transform >> .orientation)
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

                    initialTransform =
                        atOrigin
                            |> withPosition (Point2d.meters 5 3)
                            |> withOrientation (Angle.radians 3.14159)

                    trajectory =
                        simulateSteps 20 (\transform -> lookAt defaultConfig transform target) initialTransform

                    initialPosition =
                        initialTransform.position

                    finalPosition =
                        trajectory
                            |> List.reverse
                            |> List.head
                            |> Maybe.map (.transform >> .position)
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
                in
                seek config source target
                    |> .linear
                    |> Expect.equal (Just config.maxAcceleration)
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


closenessThreshold : Length.Length
closenessThreshold =
    Length.meters 0.1


seekFuzzTests : Test
seekFuzzTests =
    describe "seek behavior - fuzz tests"
        [ fuzz2 transformFuzzer point2dFuzzer "always respects max acceleration limits" <|
            \source targetPos ->
                let
                    result =
                        seek defaultConfig source targetPos
                in
                case result.linear of
                    Just acceleration ->
                        acceleration |> expectLessThanOrEqualTo defaultConfig.maxAcceleration

                    Nothing ->
                        if Point2d.distanceFrom source.position targetPos |> Quantity.lessThan closenessThreshold then
                            Expect.pass

                        else
                            Expect.fail "Expected linear acceleration"
        , fuzz2 point2dFuzzer point2dFuzzer "produces angular steering when not aligned with target" <|
            \sourcePos targetPos ->
                if Point2d.distanceFrom sourcePos targetPos |> Quantity.greaterThan closenessThreshold then
                    let
                        targetDirection =
                            Direction2d.from sourcePos targetPos

                        misalignedOrientation =
                            case targetDirection of
                                Just dir ->
                                    Direction2d.toAngle dir |> Quantity.plus (Angle.radians 1.57)

                                Nothing ->
                                    Angle.radians 1.57

                        transform =
                            withPosition sourcePos

                        source =
                            atOrigin
                                |> withPosition sourcePos
                                |> withOrientation misalignedOrientation
                    in
                    seek defaultConfig source targetPos
                        |> .angular
                        |> Expect.notEqual Nothing

                else
                    Expect.pass
        ]


seekIntegrationTests : Test
seekIntegrationTests =
    describe "seek behavior - integration tests"
        [ test "consistently moves toward target over multiple steps" <|
            \_ ->
                let
                    target =
                        Point2d.meters 10 0

                    initialTransform =
                        atOrigin |> withPosition (Point2d.meters -5 0)

                    trajectory =
                        simulateSteps 20 (\transform -> seek defaultConfig transform target) initialTransform

                    distances =
                        List.map (distanceToTarget target) trajectory
                in
                expectMonotonicallyDecreasing distances
        , test "gets closer to target over time" <|
            \_ ->
                let
                    target =
                        Point2d.meters 5 0

                    initialTransform =
                        atOrigin |> withPosition (Point2d.meters 0 0)

                    initialDistance =
                        Point2d.distanceFrom initialTransform.position target

                    trajectory =
                        simulateSteps 10 (\transform -> seek defaultConfig transform target) initialTransform

                    finalDistance =
                        trajectory
                            |> List.reverse
                            |> List.head
                            |> Maybe.map (.transform >> .position >> Point2d.distanceFrom target)
                            |> Maybe.withDefault initialDistance
                in
                finalDistance |> expectLessThan initialDistance
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
                            |> withVelocity (Speed.metersPerSecond 5)

                    target =
                        Point2d.meters 5 0
                in
                arrive config source target
                    |> .linear
                    |> Maybe.map (expectLessThan Quantity.zero)
                    |> Maybe.withDefault (Expect.fail "Expected linear steering")
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
                    ( Just arriveAccel, Just seekAccel ) ->
                        arriveAccel |> expectLessThanOrEqualTo seekAccel

                    _ ->
                        Expect.fail "Both behaviors should produce acceleration"
        ]


arriveFuzzTests : Test
arriveFuzzTests =
    describe "arrive behavior - fuzz tests"
        [ fuzz2 transformFuzzer point2dFuzzer "always respects max acceleration limits" <|
            \source targetPos ->
                let
                    result =
                        arrive defaultConfig source targetPos
                in
                case result.linear of
                    Just acceleration ->
                        Quantity.abs acceleration |> expectLessThanOrEqualTo defaultConfig.maxAcceleration

                    Nothing ->
                        if Point2d.distanceFrom source.position targetPos |> Quantity.lessThan closenessThreshold then
                            Expect.pass

                        else
                            Expect.fail "Expected linear acceleration"
        , fuzz2 point2dFuzzer point2dFuzzer "produces angular steering when not aligned with target" <|
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
                    in
                    arrive defaultConfig source targetPos
                        |> .angular
                        |> Expect.notEqual Nothing

                else
                    Expect.pass
        , fuzz transformFuzzer "produces deceleration when moving fast toward close target" <|
            \source ->
                let
                    fastVelocity =
                        Speed.metersPerSecond 8

                    closeTarget =
                        source.position |> Point2d.translateBy (Vector2d.meters 2 0)

                    fastSource =
                        { source | velocity = fastVelocity }
                in
                arrive defaultConfig fastSource closeTarget
                    |> .linear
                    |> Maybe.map (expectLessThan Quantity.zero)
                    |> Maybe.withDefault (Expect.fail "Expected linear steering")
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

                    initialTransform =
                        atOrigin

                    trajectory =
                        simulateSteps 100 (\transform -> arrive defaultConfig transform target) initialTransform

                    measurements =
                        List.map .transform trajectory

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
                            |> List.map .velocity
                            -- Stop measuring before expected oscillation around the target velocity begins
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

                    initialTransform =
                        atOrigin |> withPosition (Point2d.meters 0 0)

                    finalDistance =
                        simulateUntilConverged (\transform targetPos -> arrive defaultConfig transform targetPos) target initialTransform
                in
                finalDistance |> expectLessThan (Length.meters 0.2)
        , test "reaches target without excessive oscillation" <|
            \_ ->
                let
                    target =
                        Point2d.meters 8 0

                    initialTransform =
                        atOrigin |> withPosition (Point2d.meters 0 0)

                    finalDistance =
                        simulateUntilConverged
                            (\transform targetPos -> arrive defaultConfig transform targetPos)
                            target
                            initialTransform
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
        [ fuzz3 transformFuzzer angleFuzzer seedFuzzer "always respects max acceleration limits" <|
            \source wanderAngle seed ->
                let
                    ( steering, _, _ ) =
                        wander defaultConfig defaultWanderConfig seed wanderAngle source
                in
                case steering.linear of
                    Just acceleration ->
                        Quantity.abs acceleration |> expectLessThanOrEqualTo defaultConfig.maxAcceleration

                    Nothing ->
                        Expect.fail "Wander should always produce linear acceleration"
        , fuzz3 transformFuzzer angleFuzzer seedFuzzer "respects max angular acceleration limits" <|
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
        , fuzz3 transformFuzzer angleFuzzer seedFuzzer "produces deterministic results for same inputs" <|
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
        simulateWanderSteps numSteps initialSeed steeringConfig initialWanderAngle initialTransform =
            let
                wanderBehavior transform ( seed, wanderAngle ) =
                    let
                        ( steering, nextWanderAngle, nextSeed ) =
                            wander steeringConfig defaultWanderConfig seed wanderAngle transform
                    in
                    ( steering, ( nextSeed, nextWanderAngle ) )

                results =
                    simulateStepsWithState numSteps wanderBehavior ( initialSeed, initialWanderAngle ) initialTransform
            in
            List.map
                (\{ step, transform, steering, state } ->
                    let
                        ( seed, wanderAngle ) =
                            state
                    in
                    { step = step
                    , transform = transform
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
                    initialTransform =
                        atOrigin

                    initialWanderAngle =
                        Angle.radians 0

                    trajectory =
                        simulateWanderSteps 30 defaultSeed defaultConfig initialWanderAngle initialTransform

                    positions =
                        trajectory |> List.map (.transform >> .position)

                    -- Check that the path isn't a straight line by measuring variance in direction
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
                    initialTransform =
                        atOrigin

                    seed =
                        Random.initialSeed 54321

                    initialWanderAngle =
                        Angle.radians 0

                    trajectory =
                        simulateWanderSteps 20 seed defaultConfig initialWanderAngle initialTransform

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
                    initialTransform =
                        atOrigin

                    seed =
                        Random.initialSeed 98765

                    initialWanderAngle =
                        Angle.radians 0

                    trajectory =
                        simulateWanderSteps 25 seed defaultConfig initialWanderAngle initialTransform

                    positions =
                        trajectory |> List.map (.transform >> .position)

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


expectLessThanOrEqualTo : Quantity number units -> Quantity number units -> Expect.Expectation
expectLessThanOrEqualTo baseline value =
    if value |> Quantity.lessThanOrEqualTo baseline then
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


applySteering : SteeringConfig2d -> Duration -> Steering2d -> Transform2d coords -> Transform2d coords
applySteering config delta steering transform =
    let
        nextVelocity =
            case steering.linear of
                Just acceleration ->
                    transform.velocity
                        |> Quantity.plus (acceleration |> Quantity.for delta)
                        |> clampVelocity config

                Nothing ->
                    transform.velocity

        nextOrientation =
            transform.orientation
                |> Quantity.plus (nextRotation |> Quantity.for delta)

        nextRotation =
            case steering.angular of
                Just angularAcceleration ->
                    transform.rotation
                        |> Quantity.plus (angularAcceleration |> Quantity.for delta)

                Nothing ->
                    transform.rotation

        angle =
            Angle.inRadians nextOrientation

        d =
            nextVelocity
                |> Quantity.for delta
                |> Length.inMeters

        offset =
            Vector2d.meters (d * cos angle) (d * sin angle)
    in
    { transform
        | position = Point2d.translateBy offset transform.position
        , orientation = nextOrientation
        , velocity = nextVelocity
        , rotation = nextRotation
    }


clampVelocity : SteeringConfig2d -> Speed -> Speed
clampVelocity { minVelocity, maxVelocity } velocity =
    Quantity.clamp minVelocity maxVelocity velocity



-- Integration


type alias SimulationStep state =
    { step : Int
    , transform : Transform2d TestCoordinates
    , steering : Steering2d
    , state : state
    }


simulateSteps :
    Int
    -> (Transform2d TestCoordinates -> Steering2d)
    -> Transform2d TestCoordinates
    -> List (SimulationStep ())
simulateSteps numSteps behaviorFn initialTransform =
    simulateStepsWithState numSteps (\transform _ -> ( behaviorFn transform, () )) () initialTransform


simulateStepsWithState :
    Int
    -> (Transform2d TestCoordinates -> state -> ( Steering2d, state ))
    -> state
    -> Transform2d TestCoordinates
    -> List (SimulationStep state)
simulateStepsWithState numSteps behaviorFn initialState initialTransform =
    simulateStepsHelper numSteps behaviorFn initialState initialTransform 0 []


simulateStepsHelper :
    Int
    -> (Transform2d TestCoordinates -> state -> ( Steering2d, state ))
    -> state
    -> Transform2d TestCoordinates
    -> Int
    -> List (SimulationStep state)
    -> List (SimulationStep state)
simulateStepsHelper remaining behaviorFn currentState currentTransform stepNum acc =
    if remaining <= 0 then
        List.reverse acc

    else
        let
            ( steering, nextState ) =
                behaviorFn currentTransform currentState

            nextTransform =
                applySteering defaultConfig (Duration.seconds 0.1) steering currentTransform

            step =
                { step = stepNum
                , transform = currentTransform
                , steering = steering
                , state = currentState
                }
        in
        simulateStepsHelper (remaining - 1) behaviorFn nextState nextTransform (stepNum + 1) (step :: acc)


distanceToTarget : Point2d Length.Meters TestCoordinates -> SimulationStep state -> Length.Length
distanceToTarget target step =
    Point2d.distanceFrom step.transform.position target


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
    (Transform2d TestCoordinates -> Point2d Length.Meters TestCoordinates -> Steering2d)
    -> Point2d Length.Meters TestCoordinates
    -> Transform2d TestCoordinates
    -> Length.Length
simulateUntilConverged behaviorFn target initialTransform =
    simulateUntilConvergedWith defaultConvergenceConfig behaviorFn target initialTransform


simulateUntilConvergedWith :
    ConvergenceConfig
    -> (Transform2d TestCoordinates -> Point2d Length.Meters TestCoordinates -> Steering2d)
    -> Point2d Length.Meters TestCoordinates
    -> Transform2d TestCoordinates
    -> Length.Length
simulateUntilConvergedWith config behaviorFn target initialTransform =
    simulateUntilConvergedHelper config behaviorFn target initialTransform 0 initialLastDistance


simulateUntilConvergedHelper :
    ConvergenceConfig
    -> (Transform2d TestCoordinates -> Point2d Length.Meters TestCoordinates -> Steering2d)
    -> Point2d Length.Meters TestCoordinates
    -> Transform2d TestCoordinates
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
