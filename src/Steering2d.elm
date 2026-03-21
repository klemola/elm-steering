module Steering2d exposing
    ( Collision2d
    , Kinematic2d
    , Steering2d
    , SteeringConfig2d
    , WanderConfig2d
    , accelerate
    , arrive
    , combine
    , combineWeighted
    , decelerate
    , flee
    , lookAt
    , lookWhereYoureGoing
    , none
    , rotate
    , rotateCounterclockwise
    , seek
    , stopAtDistance
    , stopRotating
    , wallAvoidance
    , wander
    )

import Acceleration exposing (Acceleration)
import Angle exposing (Angle)
import AngularAcceleration exposing (AngularAcceleration)
import AngularSpeed exposing (AngularSpeed)
import Direction2d exposing (Direction2d)
import Duration
import Length exposing (Length)
import Point2d exposing (Point2d)
import Quantity exposing (Quantity(..))
import Random
import Speed exposing (Speed)
import Vector2d exposing (Vector2d)


type alias Steering2d coords =
    { linear : Maybe (Vector2d Acceleration.MetersPerSecondSquared coords)
    , angular : Maybe AngularAcceleration
    }


type alias Kinematic2d coords =
    { position : Point2d Length.Meters coords
    , velocity : Vector2d Speed.MetersPerSecond coords
    , orientation : Angle
    , rotation : AngularSpeed
    }


type alias SteeringConfig2d =
    { minVelocity : Speed
    , maxVelocity : Speed
    , maxDeceleration : Acceleration
    , maxAcceleration : Acceleration
    , maxAngularAcceleration : AngularAcceleration
    , maxRotation : AngularSpeed
    }


none : Steering2d coords
none =
    { linear = Nothing
    , angular = Nothing
    }


accelerate : SteeringConfig2d -> Kinematic2d coords -> Steering2d coords
accelerate { maxAcceleration } source =
    let
        velocityDirection =
            Vector2d.direction source.velocity
                |> Maybe.withDefault (Direction2d.fromAngle source.orientation)
    in
    { linear = Just (Vector2d.withLength maxAcceleration velocityDirection)
    , angular = Nothing
    }


decelerate : SteeringConfig2d -> Kinematic2d coords -> Steering2d coords
decelerate config source =
    case Vector2d.direction source.velocity of
        Just velocityDirection ->
            let
                currentSpeed =
                    Vector2d.length source.velocity

                decelerationMagnitude =
                    reachTargetVelocity config currentSpeed Quantity.zero

                decelerationVector =
                    Vector2d.withLength decelerationMagnitude velocityDirection
            in
            { linear = Just decelerationVector
            , angular = Nothing
            }

        Nothing ->
            none


stopAtDistance :
    SteeringConfig2d
    -> Length
    -> Length
    -> Kinematic2d coords
    -> Steering2d coords
stopAtDistance config distanceFromTarget threshold source =
    let
        currentSpeed =
            Vector2d.length source.velocity

        stopDistance =
            distanceFromTarget |> Quantity.minus threshold

        accelerationMagnitude =
            if stopDistance |> Quantity.lessThanOrEqualToZero then
                reachTargetVelocity config currentSpeed Quantity.zero

            else
                accelerateToZeroOverDistance config currentSpeed stopDistance

        forwardDirection =
            Direction2d.fromAngle source.orientation

        accelerationVector =
            Vector2d.withLength accelerationMagnitude forwardDirection
    in
    { linear = Just accelerationVector
    , angular = Nothing
    }


rotate : SteeringConfig2d -> Steering2d coords
rotate { maxAngularAcceleration } =
    { linear = Nothing
    , angular = Just (Quantity.negate maxAngularAcceleration)
    }


rotateCounterclockwise : SteeringConfig2d -> Steering2d coords
rotateCounterclockwise { maxAngularAcceleration } =
    { linear = Nothing
    , angular = Just maxAngularAcceleration
    }


stopRotating : SteeringConfig2d -> AngularSpeed -> Steering2d coords
stopRotating config currentRotation =
    { linear = Nothing
    , angular = Just <| reachTargetAngularVelocity config currentRotation Quantity.zero
    }


lookWhereYoureGoing : SteeringConfig2d -> Kinematic2d coords -> Steering2d coords
lookWhereYoureGoing config source =
    if Vector2d.length source.velocity |> Quantity.lessThan (Speed.metersPerSecond 0.1) then
        none

    else
        let
            targetOrientation =
                Vector2d.direction source.velocity
                    |> Maybe.map Direction2d.toAngle
                    |> Maybe.withDefault source.orientation
        in
        align config
            { currentRotation = source.rotation
            , currentOrientation = source.orientation
            , targetOrientation = targetOrientation
            }


seek : SteeringConfig2d -> Kinematic2d coords -> Point2d Length.Meters coords -> Steering2d coords
seek config source target =
    case Direction2d.from source.position target of
        Just directionToTarget ->
            { linear = Just (Vector2d.withLength config.maxAcceleration directionToTarget)
            , angular = Nothing
            }

        Nothing ->
            none


flee : SteeringConfig2d -> Kinematic2d coords -> Point2d Length.Meters coords -> Steering2d coords
flee config source target =
    case Direction2d.from target source.position of
        Just directionFromTarget ->
            { linear = Just (Vector2d.withLength config.maxAcceleration directionFromTarget)
            , angular = Nothing
            }

        Nothing ->
            none


arrive : SteeringConfig2d -> Kinematic2d coords -> Point2d Length.Meters coords -> Steering2d coords
arrive config source target =
    case Direction2d.from source.position target of
        Nothing ->
            none

        Just directionToTarget ->
            let
                targetRadius =
                    Length.meters 0.5

                slowingRadius =
                    Length.meters 8.0

                timeToTarget =
                    Duration.seconds 0.1

                distance =
                    Point2d.distanceFrom source.position target
            in
            if distance |> Quantity.lessThan targetRadius then
                decelerate config source

            else
                let
                    targetSpeed =
                        if distance |> Quantity.lessThan slowingRadius then
                            config.maxVelocity |> Quantity.multiplyBy (Quantity.ratio distance slowingRadius)

                        else
                            config.maxVelocity

                    targetVelocity =
                        Vector2d.withLength targetSpeed directionToTarget

                    acceleration =
                        targetVelocity
                            |> Vector2d.minus source.velocity
                            |> Vector2d.per timeToTarget

                    finalAcceleration =
                        if Vector2d.length acceleration |> Quantity.greaterThan config.maxAcceleration then
                            Vector2d.scaleTo config.maxAcceleration acceleration

                        else
                            acceleration
                in
                { linear = Just finalAcceleration
                , angular = Nothing
                }


lookAt : SteeringConfig2d -> Kinematic2d coords -> Point2d Length.Meters coords -> Steering2d coords
lookAt config source target =
    align config
        { currentRotation = source.rotation
        , currentOrientation = source.orientation
        , targetOrientation =
            Direction2d.from source.position target
                |> Maybe.withDefault Direction2d.positiveX
                |> Direction2d.toAngle
        }


type alias WanderConfig2d =
    { distance : Length
    , radius : Length
    , rate : Float
    }


wander :
    SteeringConfig2d
    -> WanderConfig2d
    -> Random.Seed
    -> Angle
    -> Kinematic2d coords
    -> ( Steering2d coords, Angle, Random.Seed )
wander steeringConfig { distance, radius, rate } seed wanderOrientation source =
    let
        ( angleDelta, nextSeed ) =
            Random.step (generateAngleDelta -rate rate) seed

        nextWanderOrientation =
            wanderOrientation |> Quantity.plus angleDelta

        targetOrientation =
            source.orientation |> Quantity.plus nextWanderOrientation

        forwardDirection =
            Direction2d.fromAngle source.orientation

        circleCenter =
            source.position
                |> Point2d.translateIn forwardDirection distance

        target =
            circleCenter |> Point2d.translateIn (Direction2d.fromAngle targetOrientation) radius

        angularSteering =
            lookAt steeringConfig source target

        forwardAcceleration =
            Vector2d.withLength steeringConfig.maxAcceleration forwardDirection
    in
    ( { linear = Just forwardAcceleration
      , angular = angularSteering.angular
      }
    , nextWanderOrientation
    , nextSeed
    )


type alias Collision2d coords =
    ( Point2d Length.Meters coords, Direction2d coords )


wallAvoidance :
    SteeringConfig2d
    -> (Point2d Length.Meters coords -> Direction2d coords -> Length -> Maybe (Collision2d coords))
    -> Length
    -> Kinematic2d coords
    -> Steering2d coords
wallAvoidance config detectCollisionFn probeDistance kinematic =
    let
        lookAheadDirection =
            Vector2d.direction kinematic.velocity
                |> Maybe.withDefault (Direction2d.fromAngle kinematic.orientation)

        whiskerAngle =
            Angle.degrees 45

        leftWhisker =
            Direction2d.rotateBy whiskerAngle lookAheadDirection

        rightWhisker =
            Direction2d.rotateBy (Quantity.negate whiskerAngle) lookAheadDirection

        sideProbeDistance =
            probeDistance |> Quantity.multiplyBy 0.5

        feelerForce : Direction2d coords -> Length -> Vector2d Acceleration.MetersPerSecondSquared coords
        feelerForce direction feelerLength =
            case detectCollisionFn kinematic.position direction feelerLength of
                Just ( collisionPoint, wallNormal ) ->
                    let
                        feelerTip =
                            kinematic.position |> Point2d.translateIn direction feelerLength

                        overshoot =
                            Point2d.distanceFrom collisionPoint feelerTip
                    in
                    Vector2d.withLength
                        (overshoot
                            |> Quantity.per (Duration.seconds 1)
                            |> Quantity.per (Duration.seconds 1)
                        )
                        wallNormal

                Nothing ->
                    Vector2d.zero

        totalForce =
            Vector2d.zero
                |> Vector2d.plus (feelerForce lookAheadDirection probeDistance)
                |> Vector2d.plus (feelerForce leftWhisker sideProbeDistance)
                |> Vector2d.plus (feelerForce rightWhisker sideProbeDistance)
    in
    if Vector2d.length totalForce |> Quantity.greaterThan Quantity.zero then
        let
            clampedForce =
                if Vector2d.length totalForce |> Quantity.greaterThan config.maxAcceleration then
                    Vector2d.scaleTo config.maxAcceleration totalForce

                else
                    totalForce
        in
        { linear = Just clampedForce
        , angular = Nothing
        }

    else
        none



-- Utilties


combine : SteeringConfig2d -> List (Steering2d coords) -> Steering2d coords
combine config steerings =
    steerings
        |> List.map (Tuple.pair 1)
        |> combineWeighted config


combineWeighted : SteeringConfig2d -> List ( Float, Steering2d coords ) -> Steering2d coords
combineWeighted config weightedSteerings =
    let
        activeSteerings =
            List.filter (\( _, s ) -> s.linear /= Nothing || s.angular /= Nothing) weightedSteerings

        ( accLinear, accAngular ) =
            List.foldl
                (\( weight, steering ) ( linearAcc, angularAcc ) ->
                    ( accumulateLinear config (abs weight) steering linearAcc
                    , accumulateAngular config (abs weight) steering angularAcc
                    )
                )
                ( Vector2d.zero, Quantity.zero )
                activeSteerings
    in
    { linear =
        if Vector2d.length accLinear |> Quantity.greaterThan Quantity.zero then
            Just accLinear

        else
            Nothing
    , angular =
        if Quantity.abs accAngular |> Quantity.greaterThan Quantity.zero then
            Just accAngular

        else
            Nothing
    }



--
-- Internals
--


accumulateLinear :
    SteeringConfig2d
    -> Float
    -> Steering2d coords
    -> Vector2d Acceleration.MetersPerSecondSquared coords
    -> Vector2d Acceleration.MetersPerSecondSquared coords
accumulateLinear config weight steering acc =
    case steering.linear of
        Nothing ->
            acc

        Just linear ->
            let
                contribution =
                    Vector2d.scaleBy weight linear

                currentMagnitude =
                    Vector2d.length acc

                remainingBudget =
                    config.maxAcceleration |> Quantity.minus currentMagnitude
            in
            if remainingBudget |> Quantity.lessThanOrEqualToZero then
                acc

            else
                let
                    contributionMagnitude =
                        Vector2d.length contribution
                in
                if contributionMagnitude |> Quantity.lessThanOrEqualTo remainingBudget then
                    acc |> Vector2d.plus contribution

                else
                    acc |> Vector2d.plus (Vector2d.scaleTo remainingBudget contribution)


accumulateAngular :
    SteeringConfig2d
    -> Float
    -> Steering2d coords
    -> AngularAcceleration
    -> AngularAcceleration
accumulateAngular config weight steering acc =
    case steering.angular of
        Nothing ->
            acc

        Just angular ->
            let
                contribution =
                    angular |> Quantity.multiplyBy weight

                currentMagnitude =
                    Quantity.abs acc

                remainingBudget =
                    config.maxAngularAcceleration |> Quantity.minus currentMagnitude
            in
            if remainingBudget |> Quantity.lessThanOrEqualToZero then
                acc

            else
                let
                    contributionMagnitude =
                        Quantity.abs contribution
                in
                if contributionMagnitude |> Quantity.lessThanOrEqualTo remainingBudget then
                    acc |> Quantity.plus contribution

                else
                    -- Scale to fill remaining budget, preserving direction
                    let
                        sign =
                            if contribution |> Quantity.greaterThanZero then
                                1

                            else
                                -1
                    in
                    acc |> Quantity.plus (remainingBudget |> Quantity.multiplyBy sign)


generateAngleDelta : Float -> Float -> Random.Generator Angle
generateAngleDelta min max =
    Random.float min max
        |> Random.map Angle.radians


isCloseToZeroVelocity : Speed -> Bool
isCloseToZeroVelocity =
    Quantity.abs >> Quantity.lessThan (Speed.metersPerSecond 0.1)


reachTargetVelocity : SteeringConfig2d -> Speed -> Speed -> Acceleration
reachTargetVelocity { maxAcceleration } currentVelocity targetVelocity =
    let
        acceleration =
            if Quantity.difference currentVelocity targetVelocity |> isCloseToZeroVelocity then
                -- With floating point precision the velocity is rarely exactly zero.
                -- With absolute comparison to target speed and a very low acceleration the
                -- steerable will hover just above and below zero, appearing to be still
                Acceleration.metersPerSecondSquared 0.1

            else
                maxAcceleration
    in
    if currentVelocity |> Quantity.lessThan targetVelocity then
        acceleration

    else
        Quantity.negate acceleration


accelerateToZeroOverDistance : SteeringConfig2d -> Speed -> Length -> Acceleration
accelerateToZeroOverDistance config currentVelocity (Quantity distanceFromTarget) =
    let
        { maxVelocity, maxDeceleration, maxAcceleration } =
            config
    in
    if distanceFromTarget < 0.1 then
        reachTargetVelocity config currentVelocity Quantity.zero

    else
        -- Linear acceleration (or deceleration) for a distance from a starting speed to final speed
        -- Original formula: a = (Vf*Vf - Vi*Vi)/(2 * d)
        -- (where Vf = final speed, Vi = starting speed, d = distance, and the result a is acceleration)
        -- ...but with `abs` on the starting speed to handle negative velocity
        let
            (Quantity startingSpeed) =
                currentVelocity

            finalSpeed =
                if distanceFromTarget > 1 && abs startingSpeed < 1 then
                    -- (Nearly) stopped before reaching the target; accelerate to move towards the target
                    Speed.inMetersPerSecond maxVelocity * 0.5

                else
                    -- Already in motion; try to achieve optimal deceleration
                    0
        in
        ((finalSpeed * finalSpeed - startingSpeed * abs startingSpeed) / (2 * distanceFromTarget))
            |> Quantity
            |> Quantity.clamp maxDeceleration maxAcceleration


reachTargetAngularVelocity : SteeringConfig2d -> AngularSpeed -> AngularSpeed -> AngularAcceleration
reachTargetAngularVelocity config currentRotation targetRotation =
    let
        timeToTarget =
            Duration.seconds 0.1
    in
    targetRotation
        |> Quantity.minus currentRotation
        |> Quantity.per timeToTarget
        |> Quantity.clamp
            (Quantity.negate config.maxAngularAcceleration)
            config.maxAngularAcceleration


align :
    SteeringConfig2d
    -> { currentRotation : AngularSpeed, currentOrientation : Angle, targetOrientation : Angle }
    -> Steering2d coords
align config { currentRotation, currentOrientation, targetOrientation } =
    let
        targetRadius =
            Angle.radians 0.017

        rotation =
            targetOrientation
                |> Quantity.minus currentOrientation
                |> Angle.normalize

        rotationSize =
            Quantity.abs rotation
    in
    if rotationSize |> Quantity.lessThan targetRadius then
        none

    else
        let
            slowRadius =
                Angle.radians 0.3

            targetRotation =
                if rotationSize |> Quantity.greaterThan slowRadius then
                    config.maxRotation

                else
                    config.maxRotation |> Quantity.multiplyBy (Quantity.ratio rotationSize slowRadius)

            targetRotationWithDirection =
                targetRotation
                    |> Quantity.times rotation
                    |> Quantity.over_ rotationSize
        in
        { linear = Nothing
        , angular = Just (reachTargetAngularVelocity config currentRotation targetRotationWithDirection)
        }
