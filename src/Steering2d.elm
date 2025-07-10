module Steering2d exposing
    ( Steering2d
    , SteeringConfig2d
    , Transform2d
    , accelerate
    , arrive
    , decelerate
    , lookAt
    , none
    , rotate
    , rotateCounterclockwise
    , seek
    , stopAtDistance
    , stopRotating
    )

import Acceleration exposing (Acceleration)
import Angle exposing (Angle)
import AngularAcceleration exposing (AngularAcceleration)
import AngularSpeed exposing (AngularSpeed)
import Direction2d
import Duration
import Length exposing (Length)
import Point2d exposing (Point2d)
import Quantity exposing (Quantity(..))
import Speed exposing (Speed)


type alias Steering2d =
    { linear : Maybe Acceleration
    , angular : Maybe AngularAcceleration
    }


type alias Transform2d coords =
    { position : Point2d Length.Meters coords
    , orientation : Angle
    , velocity : Speed
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


none : Steering2d
none =
    { linear = Nothing
    , angular = Nothing
    }


accelerate : SteeringConfig2d -> Steering2d
accelerate { maxAcceleration } =
    { linear = Just maxAcceleration
    , angular = Nothing
    }


decelerate : SteeringConfig2d -> Speed -> Steering2d
decelerate config currentVelocity =
    { linear = Just <| reachTargetVelocity config currentVelocity Quantity.zero
    , angular = Nothing
    }


stopAtDistance : SteeringConfig2d -> Length -> Length -> Speed -> Steering2d
stopAtDistance config distanceFromTarget threshold currentVelocity =
    let
        stopDistance =
            distanceFromTarget |> Quantity.minus threshold

        nextAcceleration =
            if stopDistance |> Quantity.lessThanOrEqualToZero then
                reachTargetVelocity config currentVelocity Quantity.zero

            else
                accelerateToZeroOverDistance config currentVelocity stopDistance
    in
    { linear = Just nextAcceleration
    , angular = Nothing
    }


rotate : SteeringConfig2d -> Steering2d
rotate { maxAngularAcceleration } =
    { linear = Nothing
    , angular = Just maxAngularAcceleration
    }


rotateCounterclockwise : SteeringConfig2d -> Steering2d
rotateCounterclockwise { maxAngularAcceleration } =
    { linear = Nothing
    , angular = Just (Quantity.negate maxAngularAcceleration)
    }


stopRotating : SteeringConfig2d -> AngularSpeed -> Steering2d
stopRotating config currentRotation =
    { linear = Nothing
    , angular = Just <| reachTargetAngularVelocity config currentRotation Quantity.zero
    }


seek : SteeringConfig2d -> Transform2d coords -> Point2d Length.Meters coords -> Steering2d
seek config source target =
    steerToward config (\_ -> config.maxAcceleration) source target


arrive : SteeringConfig2d -> Transform2d coords -> Point2d Length.Meters coords -> Steering2d
arrive config source target =
    steerToward config (arriveAcceleration config source) source target


lookAt : SteeringConfig2d -> Transform2d coords -> Point2d Length.Meters coords -> Steering2d
lookAt config source target =
    align config
        { currentRotation = source.rotation
        , currentOrientation = source.orientation
        , targetOrientation =
            Direction2d.from source.position target |> Maybe.withDefault Direction2d.positiveX |> Direction2d.toAngle
        }



--
-- Internals
--


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
                -- car will hover just above and below zero, appearing to be still
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
    -> Steering2d
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


steerToward :
    SteeringConfig2d
    -> (Length -> Acceleration)
    -> Transform2d coords
    -> Point2d Length.Meters coords
    -> Steering2d
steerToward config accelerationFn source target =
    case Direction2d.from source.position target of
        -- The direction is only validated, not used
        Just _ ->
            { linear =
                Point2d.distanceFrom source.position target
                    |> accelerationFn
                    |> Just
            , angular = lookAt config source target |> .angular
            }

        Nothing ->
            none


arriveAcceleration : SteeringConfig2d -> Transform2d coords -> Length -> Acceleration
arriveAcceleration config source distance =
    let
        timeToTarget =
            Duration.seconds 0.1

        slowingRadius =
            Length.meters 5.0

        targetSpeed =
            if distance |> Quantity.greaterThan slowingRadius then
                config.maxVelocity

            else
                config.maxVelocity |> Quantity.multiplyBy (Quantity.ratio distance slowingRadius)
    in
    targetSpeed
        |> Quantity.minus source.velocity
        |> Quantity.per timeToTarget
        |> Quantity.clamp (Quantity.negate config.maxAcceleration) config.maxAcceleration
