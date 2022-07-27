package DynamicSandBox
  import Modelica.SIunits;

  partial model Body2D
    parameter Modelica.SIunits.Length L = 1;
    parameter Modelica.SIunits.Angle phi0 = 0;
    parameter Real Color[3] = {255, 0, 0};
    parameter Modelica.Mechanics.MultiBody.Frames.Orientation orientation = Modelica.Mechanics.MultiBody.Frames.axesRotations({1, 2, 3}, {0, 0, 0}, {0, 0, 0});
    Modelica.SIunits.Position X;
    Modelica.SIunits.Position Y;
    Modelica.SIunits.Angle Phi(start = phi0);
    KinematicOutput Body;
  equation
    Body.X = X;
    Body.Y = Y;
    Body.Phi = Phi;
  end Body2D;

  connector KinematicOutput
    output Modelica.SIunits.Position X;
    output Modelica.SIunits.Position Y;
    output Modelica.SIunits.Angle Phi;
  end KinematicOutput;

  connector KinematicInput
    input Modelica.SIunits.Position X;
    input Modelica.SIunits.Position Y;
    input Modelica.SIunits.Angle Phi;
  end KinematicInput;

  model Seat2D
    parameter Modelica.SIunits.Position Xp = 0;
    parameter Modelica.SIunits.Position Yp = 0;
    parameter Modelica.SIunits.Position Xb = 0;
    parameter Modelica.SIunits.Position Yb = 0;
    parameter Modelica.Mechanics.MultiBody.Frames.Orientation orientation = Modelica.Mechanics.MultiBody.Frames.axesRotations({1, 2, 3}, {0, 0, 0}, {0, 0, 0});
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape Seatshape(shapeType = "cylinder", length = 0.25, width = 0.1, height = 0.1, lengthDirection = {0, 0, 1}, widthDirection = {0, 1, 0}, color = {0, 0, 0}, specularCoefficient = 0.5, r = {Xp, Yp, 0}, R = orientation, r_shape = {0, 0, -0.1});
    KinematicInput Body;
    Modelica.SIunits.Force Rx;
    Modelica.SIunits.Force Ry;
    DynamicOutput Body_out;
  equation
    Xp = Body.X + Xb * cos(Body.Phi) - Yb * sin(Body.Phi);
    Yp = Body.Y + Xb * sin(Body.Phi) + Yb * cos(Body.Phi);
    Body_out.Xp = Xp;
    Body_out.Yp = Yp;
    Body_out.Fx = Rx;
    Body_out.Fy = Ry;
    Body_out.M = 0;
  end Seat2D;

  model PalkaSOporoi
    parameter Modelica.SIunits.Length L1 = 4;
    parameter Modelica.SIunits.Angle phi10 = 0;
    parameter Modelica.SIunits.Mass m1 = 2;
    parameter Modelica.SIunits.MomentOfInertia I1 = m1 * L1 ^ 2 / 12;
    Rod2D Palka(phi0 = phi10, L = L1, m = m1, I = I1);
    Seat2D Opora(Xp = 1, Yp = 3, Xb = -L1 / 2, Yb = 0);
    FreeEnd KonecPalki;
  equation
    connect(Palka.Body, Opora.Body);
    connect(Palka.A, Opora.Body_out);
    connect(Palka.B, KonecPalki.Body_out);
    annotation(
      experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-6, Interval = 0.0002));
  end PalkaSOporoi;

  model Joint2D
    parameter Modelica.SIunits.Position Xb1 = 0;
    parameter Modelica.SIunits.Position Yb1 = 0;
    parameter Modelica.SIunits.Position Xb2 = 0;
    parameter Modelica.SIunits.Position Yb2 = 0;
    parameter Modelica.Mechanics.MultiBody.Frames.Orientation orientation = Modelica.Mechanics.MultiBody.Frames.axesRotations({1, 2, 3}, {0, 0, 0}, {0, 0, 0});
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape Jointshape(shapeType = "cylinder", length = 0.25, width = 0.1, height = 0.1, lengthDirection = {0, 0, 1}, widthDirection = {0, 1, 0}, color = {0, 0, 0}, specularCoefficient = 0.5, r = {XSh, YSh, 0}, R = orientation, r_shape = {0, 0, -0.1});
    Modelica.SIunits.Position XSh;
    Modelica.SIunits.Position YSh;
    KinematicInput Body1;
    KinematicInput Body2;
    Modelica.SIunits.Force Rx;
    Modelica.SIunits.Force Ry;
    DynamicOutput Body1_out;
    DynamicOutput Body2_out;
  equation
    XSh = Body1.X + Xb1 * cos(Body1.Phi) - Yb1 * sin(Body1.Phi);
    YSh = Body1.Y + Xb1 * sin(Body1.Phi) + Yb1 * cos(Body1.Phi);
    XSh = Body2.X + Xb2 * cos(Body2.Phi) - Yb2 * sin(Body2.Phi);
    YSh = Body2.Y + Xb2 * sin(Body2.Phi) + Yb2 * cos(Body2.Phi);
    Body1_out.Xp = XSh;
    Body1_out.Yp = YSh;
    Body1_out.Fx = Rx;
    Body1_out.Fy = Ry;
    Body1_out.M = 0;
    Body2_out.Xp = XSh;
    Body2_out.Yp = YSh;
    Body2_out.Fx = -Rx;
    Body2_out.Fy = -Ry;
    Body2_out.M = 0;
  end Joint2D;

  model DvePalkiSOporoi
    parameter Modelica.SIunits.Length L1 = 4;
    parameter Modelica.SIunits.Length L2 = 2;
    parameter Modelica.SIunits.Angle phi10 = 1;
    parameter Modelica.SIunits.Angle phi20 = -1;
    parameter Modelica.SIunits.Mass m1 = 2;
    parameter Modelica.SIunits.Mass m2 = 5;
    parameter Modelica.SIunits.MomentOfInertia I1 = m1 * L1 ^ 2 / 12;
    parameter Modelica.SIunits.MomentOfInertia I2 = m2 * L2 ^ 2 / 12;
    Rod2D Palka1(phi0 = phi10, L = L1, m = m1, I = I1);
    Seat2D Opora(Xp = 1, Yp = 3, Xb = -L1 / 2, Yb = 0);
    Rod2D Palka2(phi0 = phi20, L = L2, m = m2, I = I2);
    Joint2D Sharnir(Xb1 = L1 / 2, Yb1 = 0, Xb2 = -L2 / 2, Yb2 = 0);
    FreeEnd KonecPalki;
  equation
    connect(Palka1.Body, Opora.Body);
    connect(Palka1.Body, Sharnir.Body1);
    connect(Palka2.Body, Sharnir.Body2);
    connect(Palka1.A, Opora.Body_out);
    connect(Palka1.B, Sharnir.Body2_out);
    connect(Palka2.B, Sharnir.Body1_out);
    connect(Palka2.A, KonecPalki.Body_out);
    annotation(
      experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-09, Interval = 0.0002));
  end DvePalkiSOporoi;

  model Slider2D
    // parametry pryamoi v prostranstve
    parameter Modelica.SIunits.Position Xp = 0;
    parameter Modelica.SIunits.Position Yp = 0;
    parameter Modelica.SIunits.Angle Phip = 0;
    // parametry polzuna v tele
    parameter Modelica.SIunits.Position Xb = 0;
    parameter Modelica.SIunits.Position Yb = 0;
    // animatsiya
    parameter Modelica.SIunits.Length BoxLength = 0.25;
    parameter Modelica.Mechanics.MultiBody.Frames.Orientation orientation = Modelica.Mechanics.MultiBody.Frames.axesRotations({1, 2, 3}, {0, 0, 0}, {0, 0, 0});
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape SliderJshape(shapeType = "cylinder", length = 0.25, width = 0.1, height = 0.1, lengthDirection = {0, 0, 1}, widthDirection = {0, 1, 0}, color = {0, 0, 0}, specularCoefficient = 0.5, r = {Xp + S * cos(Phip), Yp + S * sin(Phip), 0}, R = orientation, r_shape = {0, 0, 0});
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape SliderBshape(shapeType = "box", length = BoxLength, width = 0.2, height = 0.2, lengthDirection = {cos(Phip), sin(Phip), 0}, widthDirection = {0, 0, 1}, color = {0, 0, 160}, specularCoefficient = 0.5, r = {Xp + (S - BoxLength / 2) * cos(Phip), Yp + (S - BoxLength / 2) * sin(Phip), 0}, R = orientation, r_shape = {0, 0, 0});
    // polozhenie polzuna na pryamoi
    Modelica.SIunits.Length S;
    KinematicInput Body;
    DynamicOutput Body_out;
    Modelica.SIunits.Force N;
  equation
    Xp + S * cos(Phip) = Body.X + Xb * cos(Body.Phi) - Yb * sin(Body.Phi);
    Yp + S * sin(Phip) = Body.Y + Xb * sin(Body.Phi) + Yb * cos(Body.Phi);
    Body_out.Xp = Xp + S * cos(Phip);
    Body_out.Yp = Yp + S * sin(Phip);
    Body_out.Fx = -N * sin(Phip);
    Body_out.Fy = N * cos(Phip);
    Body_out.M = 0;
  end Slider2D;

  model SliderTest
    parameter Modelica.SIunits.Length L1 = 1;
    parameter Modelica.SIunits.Length L2 = 3;
    parameter Modelica.SIunits.AngularVelocity Omega1 = 2;
    parameter Modelica.SIunits.AngularVelocity Omega2 = 10;
    parameter Modelica.SIunits.Angle phi10 = 0;
    parameter Modelica.SIunits.Angle phi20 = 0;
    parameter Modelica.SIunits.Position Xp0 = L1 * cos(phi10) + L2 * cos(phi20);
    parameter Modelica.SIunits.Position Yp0 = L1 * sin(phi10) + L2 * sin(phi20);
    parameter Modelica.SIunits.Mass m1 = 1;
    parameter Modelica.SIunits.Mass m2 = 3;
    parameter Modelica.SIunits.MomentOfInertia I1 = m1 * L1 ^ 2 / 12;
    parameter Modelica.SIunits.MomentOfInertia I2 = m2 * L2 ^ 2 / 12;
    Rod2D Palka1(phi0 = phi10, L = L1, m = m1, I = I1);
    Seat2D Opora(Xp = 0, Yp = 0, Xb = -L1 / 2, Yb = 0);
    Rod2D Palka2(phi0 = phi20, L = L2, m = m2, I = I2);
    Joint2D Sharnir(Xb1 = L1 / 2, Yb1 = 0, Xb2 = -L2 / 2, Yb2 = 0);
    ForcedSlider2D Polzun(Xb = L2 / 2, Yb = 0, Xp = Xp0, Yp = Yp0, Phip = 0.5, coef = 0.1);
  equation
    connect(Palka1.Body, Opora.Body);
    connect(Palka1.Body, Sharnir.Body1);
    connect(Palka2.Body, Sharnir.Body2);
    connect(Palka2.Body, Polzun.Body);
    connect(Palka1.A, Opora.Body_out);
    connect(Palka1.B, Sharnir.Body1_out);
    connect(Palka2.A, Sharnir.Body2_out);
    connect(Palka2.B, Polzun.Body_out);
    annotation(
      experiment(StartTime = 0, StopTime = 20, Tolerance = 1e-06, Interval = 0.0004));
  end SliderTest;

  model RollCircleOnLine
    // parametry pryamoi v prostranstve
    parameter Modelica.SIunits.Position Xp = 0;
    parameter Modelica.SIunits.Position Yp = 0;
    parameter Modelica.SIunits.Angle Phip = 0;
    // animatsiya
    parameter Modelica.SIunits.Length R = 1;
    parameter Modelica.Mechanics.MultiBody.Frames.Orientation orientation = Modelica.Mechanics.MultiBody.Frames.axesRotations({1, 2, 3}, {0, 0, 0}, {0, 0, 0});
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape Boxshape(shapeType = "box", length = 6 * R, width = 0.3, height = 0.05, lengthDirection = {cos(Phip), sin(Phip), 0}, widthDirection = {0, 0, 1}, color = {160, 0, 0}, specularCoefficient = 0.5, r = {Xp - 3 * R * cos(Phip) + 0.025 * sin(Phip), Yp - 3 * R * sin(Phip) - 0.025 * cos(Phip), 0}, R = orientation, r_shape = {0, 0, 0});
    // polozhenie polzuna na pryamoi
    Modelica.SIunits.Length S;
    KinematicInput Body;
    DynamicOutput Body_out;
    Modelica.SIunits.Force N;
    Modelica.SIunits.Force Ftr;
  equation
    Xp + S * cos(Phip) - R * sin(Phip) = Body.X;
    Yp + S * sin(Phip) + R * cos(Phip) = Body.Y;
    der(S) = -R * der(Body.Phi);
    Body_out.Xp = Xp + S * cos(Phip);
    Body_out.Yp = Yp + S * sin(Phip);
    Body_out.Fx = Ftr * cos(Phip) - N * sin(Phip);
    Body_out.Fy = Ftr * sin(Phip) + N * cos(Phip);
    Body_out.M = 0;
  end RollCircleOnLine;

  model Rod2D
    extends DynamicSandBox.TwoPortBody2D;
    parameter Modelica.SIunits.Length L = 1;
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape Bodyshape(shapeType = "box", length = L, width = 0.05, height = 0.05, lengthDirection = {cos(Phi), sin(Phi), 0}, widthDirection = {0, 0, 1}, color = Color, specularCoefficient = 0.5, r = {X - L / 2 * cos(Phi), Y - L / 2 * sin(Phi), 0}, R = orientation, r_shape = {0, 0, 0});
  end Rod2D;

  model Wheel2D
    extends DynamicSandBox.TwoPortBody2D;
    parameter Modelica.SIunits.Length R = 1;
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape Bodyshape1(shapeType = "cylinder", length = 0.05, width = 2 * R, height = 2 * R, lengthDirection = {0, 0, 1}, widthDirection = {cos(Phi), sin(Phi), 0}, color = Color, specularCoefficient = 0.5, r = {X, Y, 0}, R = orientation, r_shape = {0, 0, 0});
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape Bodyshape2(shapeType = "box", length = 0.1, width = R, height = R, lengthDirection = {0, 0, 1}, widthDirection = {cos(Phi), sin(Phi), 0}, color = Color * 0.7, specularCoefficient = 0.5, r = {X, Y, 0}, R = orientation, r_shape = {0, 0, 0});
  end Wheel2D;

  model RollTest
    parameter Modelica.SIunits.Length L1 = 1;
    parameter Modelica.SIunits.Length L2 = 3;
    parameter Modelica.SIunits.Length R1 = 1;
    parameter Modelica.SIunits.AngularVelocity Omega1 = 2;
    parameter Modelica.SIunits.AngularVelocity Omega2 = 10;
    parameter Modelica.SIunits.Angle phi10 = 0;
    parameter Modelica.SIunits.Angle phi20 = 0;
    parameter Modelica.SIunits.Angle phi30 = 0;
    parameter Modelica.SIunits.Angle phip = 0.5;
    parameter Modelica.SIunits.Position XO = -3;
    parameter Modelica.SIunits.Position YO = 0;
    parameter Modelica.SIunits.Position Xp0 = XO + L1 * cos(phi10) + L2 * cos(phi20) + R1 * sin(phip) - 3 * R1 / 4 * cos(phi30);
    parameter Modelica.SIunits.Position Yp0 = YO + L1 * sin(phi10) + L2 * sin(phi20) - R1 * cos(phip) - 3 * R1 / 4 * sin(phi30);
    parameter Modelica.SIunits.Mass m1 = 1;
    parameter Modelica.SIunits.Mass m2 = 3;
    parameter Modelica.SIunits.Mass m3 = 5;
    parameter Modelica.SIunits.MomentOfInertia I1 = m1 * L1 ^ 2 / 12;
    parameter Modelica.SIunits.MomentOfInertia I2 = m2 * L2 ^ 2 / 12;
    parameter Modelica.SIunits.MomentOfInertia I3 = m2 * R1 ^ 2 / 2;
    Rod2D Palka1(phi0 = phi10, L = L1, m = m1, I = I1);
    Seat2D Opora(Xp = XO, Yp = YO, Xb = -L1 / 2, Yb = 0);
    Rod2D Palka2(phi0 = phi20, L = L2, m = m2, I = I2);
    Joint2D Sharnir(Xb1 = L1 / 2, Yb1 = 0, Xb2 = -L2 / 2, Yb2 = 0);
    Wheel2D Koleso(phi0 = phi30, R = R1, m = m3, I = I3);
    Joint2D Sharnir2(Xb1 = L2 / 2, Yb1 = 0, Xb2 = 3 * R1 / 4, Yb2 = 0);
    RollCircleOnLine Roll(Xp = Xp0, Yp = Yp0, Phip = phip, R = R1);
  equation
    connect(Palka1.Body, Opora.Body);
    connect(Palka1.Body, Sharnir.Body1);
    connect(Palka2.Body, Sharnir.Body2);
    connect(Palka2.Body, Sharnir2.Body1);
    connect(Koleso.Body, Sharnir2.Body2);
    connect(Koleso.Body, Roll.Body);
    connect(Palka1.A, Opora.Body_out);
    connect(Palka1.B, Sharnir.Body1_out);
    connect(Palka2.A, Sharnir.Body2_out);
    connect(Palka2.B, Sharnir2.Body1_out);
    connect(Koleso.A, Sharnir2.Body2_out);
    connect(Koleso.B, Roll.Body_out);
    annotation(
      experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-06, Interval = 0.0002));
  end RollTest;

  partial model TwoPortBody2D
    extends Gravity;
    parameter Modelica.SIunits.Length L = 1;
    parameter Modelica.SIunits.Angle phi0 = 0;
    parameter Modelica.SIunits.Position X0 = 0;
    parameter Modelica.SIunits.Position Y0 = 0;
    parameter Real Color[3] = {255, 0, 0};
    parameter Modelica.Mechanics.MultiBody.Frames.Orientation orientation = Modelica.Mechanics.MultiBody.Frames.axesRotations({1, 2, 3}, {0, 0, 0}, {0, 0, 0});
    Modelica.SIunits.Position X(start = X0);
    Modelica.SIunits.Position Y(start = Y0);
    Modelica.SIunits.Angle Phi(start = phi0, stateSelect = StateSelect.prefer);
    KinematicOutput Body;
    parameter Modelica.SIunits.Mass m = 1;
    parameter Modelica.SIunits.MomentOfInertia I = 1;
    parameter Modelica.SIunits.Velocity Vx0 = 0;
    parameter Modelica.SIunits.Velocity Vy0 = 0;
    parameter Modelica.SIunits.AngularVelocity Omega0 = 0;
    Modelica.SIunits.Velocity Vx(start = Vx0, stateSelect = StateSelect.prefer);
    Modelica.SIunits.Velocity Vy(start = Vy0, stateSelect = StateSelect.prefer);
    Modelica.SIunits.Acceleration Wx;
    Modelica.SIunits.Acceleration Wy;
    Modelica.SIunits.AngularVelocity Omega(start = Omega0, stateSelect = StateSelect.prefer);
    Modelica.SIunits.AngularAcceleration Epsilon;
    Modelica.SIunits.Position CA[2];
    Modelica.SIunits.Position CB[2];
    DynamicInput A;
    DynamicInput B;
  equation
    Body.X = X;
    Body.Y = Y;
    Body.Phi = Phi;
    der(X) = Vx;
    der(Y) = Vy;
    der(Vx) = Wx;
    der(Vy) = Wy;
    der(Phi) = Omega;
    der(Omega) = Epsilon;
    CA = {A.Xp - X, A.Yp - Y};
    CB = {B.Xp - X, B.Yp - Y};
    m * Wx = A.Fx + B.Fx;
    m * Wy = A.Fy + B.Fy - m * g;
    I * Epsilon = A.M + B.M + CA[1] * A.Fy - CA[2] * A.Fx + CB[1] * B.Fy - CB[2] * B.Fx;
  end TwoPortBody2D;

  connector DynamicOutput
    output Modelica.SIunits.Position Xp;
    output Modelica.SIunits.Position Yp;
    output Modelica.SIunits.Force Fx;
    output Modelica.SIunits.Force Fy;
    output Modelica.SIunits.MomentOfForce M;
  end DynamicOutput;

  connector DynamicInput
    input Modelica.SIunits.Position Xp;
    input Modelica.SIunits.Position Yp;
    input Modelica.SIunits.Force Fx;
    input Modelica.SIunits.Force Fy;
    input Modelica.SIunits.MomentOfForce M;
  end DynamicInput;

  partial model Gravity
    parameter Modelica.SIunits.Acceleration g = 9.81;
  end Gravity;

  model FreeEnd
    DynamicOutput Body_out;
  equation
    Body_out.Xp = 0;
    Body_out.Yp = 0;
    Body_out.Fx = 0;
    Body_out.Fy = 0;
    Body_out.M = 0;
  end FreeEnd;

  partial model ThreePortBody2D
    extends Gravity;
    parameter Modelica.SIunits.Length L = 1;
    parameter Modelica.SIunits.Angle phi0 = 0;
    parameter Modelica.SIunits.Position X0 = 0;
    parameter Modelica.SIunits.Position Y0 = 0;
    parameter Real Color[3] = {255, 0, 0};
    parameter Modelica.Mechanics.MultiBody.Frames.Orientation orientation = Modelica.Mechanics.MultiBody.Frames.axesRotations({1, 2, 3}, {0, 0, 0}, {0, 0, 0});
    Modelica.SIunits.Position X(start = X0);
    Modelica.SIunits.Position Y(start = Y0);
    Modelica.SIunits.Angle Phi(start = phi0, stateSelect = StateSelect.prefer);
    KinematicOutput Body;
    parameter Modelica.SIunits.Mass m = 1;
    parameter Modelica.SIunits.MomentOfInertia I = 1;
    parameter Modelica.SIunits.Velocity Vx0 = 0;
    parameter Modelica.SIunits.Velocity Vy0 = 0;
    parameter Modelica.SIunits.AngularVelocity Omega0 = 0;
    Modelica.SIunits.Velocity Vx(start = Vx0, stateSelect = StateSelect.prefer);
    Modelica.SIunits.Velocity Vy(start = Vy0, stateSelect = StateSelect.prefer);
    Modelica.SIunits.Acceleration Wx;
    Modelica.SIunits.Acceleration Wy;
    Modelica.SIunits.AngularVelocity Omega(start = Omega0, stateSelect = StateSelect.prefer);
    Modelica.SIunits.AngularAcceleration Epsilon;
    Modelica.SIunits.Position CA[2];
    Modelica.SIunits.Position CB[2];
    Modelica.SIunits.Position CD[2];
    DynamicInput A;
    DynamicInput B;
    DynamicInput D;
  equation
    Body.X = X;
    Body.Y = Y;
    Body.Phi = Phi;
    der(X) = Vx;
    der(Y) = Vy;
    der(Vx) = Wx;
    der(Vy) = Wy;
    der(Phi) = Omega;
    der(Omega) = Epsilon;
    CA = {A.Xp - X, A.Yp - Y};
    CB = {B.Xp - X, B.Yp - Y};
    CD = {D.Xp - X, D.Yp - Y};
    m * Wx = A.Fx + B.Fx + D.Fx;
    m * Wy = A.Fy + B.Fy + D.Fy - m * g;
    I * Epsilon = A.M + B.M + D.M + CA[1] * A.Fy - CA[2] * A.Fx + CB[1] * B.Fy - CB[2] * B.Fx + CD[1] * D.Fy - CD[2] * D.Fx;
  end ThreePortBody2D;

  model ThreePortWheel
    extends DynamicSandBox.ThreePortBody2D;
    parameter Modelica.SIunits.Length R = 1;
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape Bodyshape1(shapeType = "cylinder", length = 0.05, width = 2 * R, height = 2 * R, lengthDirection = {0, 0, 1}, widthDirection = {cos(Phi), sin(Phi), 0}, color = Color, specularCoefficient = 0.5, r = {X, Y, 0}, R = orientation, r_shape = {0, 0, 0});
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape Bodyshape2(shapeType = "box", length = 0.1, width = R, height = R, lengthDirection = {0, 0, 1}, widthDirection = {cos(Phi), sin(Phi), 0}, color = Color * 0.7, specularCoefficient = 0.5, r = {X, Y, 0}, R = orientation, r_shape = {0, 0, 0});
  end ThreePortWheel;

  partial model ThreePortRod2D
    extends DynamicSandBox.ThreePortBody2D;
    parameter Modelica.SIunits.Length L = 1;
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape Bodyshape(shapeType = "box", length = L, width = 0.05, height = 0.05, lengthDirection = {cos(Phi), sin(Phi), 0}, widthDirection = {0, 0, 1}, color = Color, specularCoefficient = 0.5, r = {X - L / 2 * cos(Phi), Y - L / 2 * sin(Phi), 0}, R = orientation, r_shape = {0, 0, 0});
  end ThreePortRod2D;

  model ThreePortWheel2D
    extends DynamicSandBox.ThreePortBody2D;
    parameter Modelica.SIunits.Length R = 1;
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape Bodyshape1(shapeType = "cylinder", length = 0.05, width = 2 * R, height = 2 * R, lengthDirection = {0, 0, 1}, widthDirection = {cos(Phi), sin(Phi), 0}, color = Color, specularCoefficient = 0.5, r = {X, Y, 0}, R = orientation, r_shape = {0, 0, 0});
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape Bodyshape2(shapeType = "box", length = 0.1, width = R, height = R, lengthDirection = {0, 0, 1}, widthDirection = {cos(Phi), sin(Phi), 0}, color = Color * 0.7, specularCoefficient = 0.5, r = {X, Y, 0}, R = orientation, r_shape = {0, 0, 0});
  end ThreePortWheel2D;

  model Clutch2D
    // parametry pryamoi v tele
    parameter Modelica.SIunits.Position Xpt = 0;
    parameter Modelica.SIunits.Position Ypt = 0;
    parameter Modelica.SIunits.Angle Phipt = 0;
    // parametry mufti v prostranstve
    parameter Modelica.SIunits.Position Xp = 0;
    parameter Modelica.SIunits.Position Yp = 0;
    // animatsiya
    parameter Modelica.SIunits.Length BoxLength = 0.25;
    parameter Modelica.Mechanics.MultiBody.Frames.Orientation orientation = Modelica.Mechanics.MultiBody.Frames.axesRotations({1, 2, 3}, {0, 0, 0}, {0, 0, 0});
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape SliderJshape(shapeType = "cylinder", length = 0.25, width = 0.1, height = 0.1, lengthDirection = {0, 0, 1}, widthDirection = {0, 1, 0}, color = {0, 0, 0}, specularCoefficient = 0.5, r = {Xp, Yp, 0}, R = orientation, r_shape = {0, 0, 0});
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape SliderBshape(shapeType = "box", length = BoxLength, width = 0.2, height = 0.2, lengthDirection = {cos(Body.Phi + Phipt), sin(Body.Phi + Phipt), 0}, widthDirection = {0, 0, 1}, color = {0, 0, 160}, specularCoefficient = 0.5, r = {Xp - BoxLength / 2 * cos(Body.Phi + Phipt), Yp - BoxLength / 2 * sin(Body.Phi + Phipt), 0}, R = orientation, r_shape = {0, 0, 0});
    // polozhenie mufti na pryamoi v tele
    Modelica.SIunits.Length S;
    KinematicInput Body;
    DynamicOutput Body_out;
    Modelica.SIunits.Force N;
  equation
    Xp = Body.X + Xpt * cos(Body.Phi) - Ypt * sin(Body.Phi) + S * cos(Body.Phi + Phipt);
    Yp = Body.Y + Xpt * sin(Body.Phi) + Ypt * cos(Body.Phi) + S * sin(Body.Phi + Phipt);
    Body_out.Xp = Xp;
    Body_out.Yp = Yp;
    Body_out.Fx = -N * sin(Body.Phi + Phipt);
    Body_out.Fy = N * cos(Body.Phi + Phipt);
    Body_out.M = 0;
  end Clutch2D;

  model ClutchTest
    parameter Modelica.SIunits.Length L1 = 1;
    parameter Modelica.SIunits.Length L2 = 4;
    parameter Modelica.SIunits.Angle phi10 = 0;
    parameter Modelica.SIunits.Angle phi20 = 0;
    parameter Modelica.SIunits.Position Xp0 = L1 * cos(phi10) + L2 / 4 * cos(phi20);
    parameter Modelica.SIunits.Position Yp0 = L1 * sin(phi10) + L2 / 4 * sin(phi20);
    parameter Modelica.SIunits.Mass m1 = 9;
    parameter Modelica.SIunits.Mass m2 = 4;
    parameter Modelica.SIunits.MomentOfInertia I1 = m1 * L1 ^ 2 / 12;
    parameter Modelica.SIunits.MomentOfInertia I2 = m2 * L2 ^ 2 / 12;
    Rod2D Palka1(phi0 = phi10, L = L1, m = m1, I = I1);
    Seat2D Opora(Xp = 0, Yp = 0, Xb = -L1 / 2, Yb = 0);
    Rod2D Palka2(phi0 = phi20, L = L2, m = m2, I = I2);
    Joint2D Sharnir(Xb1 = L1 / 2, Yb1 = 0, Xb2 = -L2 / 2, Yb2 = 0);
    Clutch2D Clutch(Xp = Xp0, Yp = Yp0, Xpt = 0, Ypt = 0, Phipt = 0);
  equation
    connect(Palka1.Body, Opora.Body);
    connect(Palka1.Body, Sharnir.Body1);
    connect(Palka2.Body, Sharnir.Body2);
    connect(Palka2.Body, Clutch.Body);
    connect(Palka1.A, Opora.Body_out);
    connect(Palka1.B, Sharnir.Body1_out);
    connect(Palka2.A, Sharnir.Body2_out);
    connect(Palka2.B, Clutch.Body_out);
    annotation(
      experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-06, Interval = 0.0002));
  end ClutchTest;

  model RollCircleOnCircle
    // parametry pryamoi v prostranstve
    parameter Modelica.SIunits.Position Xp = 0;
    parameter Modelica.SIunits.Position Yp = 0;
    parameter Modelica.SIunits.Length Ro = 1;
    // animatsiya
    parameter Modelica.SIunits.Length R = 1;
    parameter Modelica.Mechanics.MultiBody.Frames.Orientation orientation = Modelica.Mechanics.MultiBody.Frames.axesRotations({1, 2, 3}, {0, 0, 0}, {0, 0, 0});
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape Pipeshape(shapeType = "pipe", length = 0.3 * R, width = 2 * Ro, height = 2 * Ro, lengthDirection = {0, 0, 1}, widthDirection = {0, 1, 0}, color = {160, 0, 0}, specularCoefficient = 0.5, r = {Xp, Yp, 0}, R = orientation, r_shape = {0, 0, 0}, extra = 0.8);
    // polozhenie polzuna na pryamoi
    Modelica.SIunits.Angle Alpha;
    KinematicInput Body;
    DynamicOutput Body_out;
    Modelica.SIunits.Force N;
    Modelica.SIunits.Force Ftr;
  equation
    Xp + (Ro + R) * sin(Alpha) = Body.X;
    Yp + (Ro + R) * cos(Alpha) = Body.Y;
    der(Alpha) * (Ro + R) = -R * der(Body.Phi);
    Body_out.Xp = Xp + Ro * sin(Alpha);
    Body_out.Yp = Yp + Ro * cos(Alpha);
    Body_out.Fx = (-Ftr * cos(Alpha)) + N * sin(Alpha);
    Body_out.Fy = Ftr * sin(Alpha) + N * cos(Alpha);
    Body_out.M = 0;
  end RollCircleOnCircle;

  model RollTwoCirclesTest
    parameter Modelica.SIunits.Length L1 = 4;
    parameter Modelica.SIunits.Length R = 4;
    parameter Modelica.SIunits.Length r = 1;
    parameter Modelica.SIunits.Angle phi10 = 1;
    parameter Modelica.SIunits.Mass m1 = 2;
    parameter Modelica.SIunits.MomentOfInertia I1 = m1 * r ^ 2 / 2;
    Wheel2D Wheel(phi0 = phi10, R = r, m = m1, I = I1, Omega0 = 0.1);
    RollCircleOnCircle Roll(Xp = 0, Yp = 0, Ro = R, R = r);
    FreeEnd KonecPalki;
  equation
    connect(Wheel.Body, Roll.Body);
    connect(Wheel.A, Roll.Body_out);
    connect(Wheel.B, KonecPalki.Body_out);
    annotation(
      experiment(StartTime = 0, StopTime = 20, Tolerance = 1e-06, Interval = 0.002));
  end RollTwoCirclesTest;

  model ThreadRR2D
    parameter Modelica.SIunits.Length L0 = 0;
    parameter Modelica.SIunits.Length R1 = 1;
    parameter Modelica.SIunits.Length R2 = 1;
    parameter Real k1 = 1;
    parameter Real k2 = 1;
    parameter Modelica.Mechanics.MultiBody.Frames.Orientation orientation = Modelica.Mechanics.MultiBody.Frames.axesRotations({1, 2, 3}, {0, 0, 0}, {0, 0, 0});
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape Threadshape(shapeType = "cylinder", length = L_ed * sqrt_L_mod ^ 2, width = 0.05, height = 0.05, lengthDirection = {sin(alpha), cos(alpha), 0}, widthDirection = {0, 0, 1}, color = {0, 0, 0}, specularCoefficient = 0.5, r = {Body2.X + k2 * R2 * cos(alpha), Body2.Y - k2 * R2 * sin(alpha), 0}, R = orientation, r_shape = {0, 0, -0.1});
    parameter Modelica.SIunits.Length L_ed = 1;
    parameter Modelica.SIunits.Length L_0 = 0;
    Real sqrt_L_mod;
    Modelica.SIunits.Angle alpha;
    KinematicInput Body1;
    KinematicInput Body2;
    Modelica.SIunits.Force T;
    DynamicOutput Body1_out;
    DynamicOutput Body2_out;
  equation
    L_ed * sqrt_L_mod ^ 2 = L_0 + k1 * R1 * (Body1.Phi + alpha) + k2 * R2 * (Body2.Phi + alpha);
    Body1.X - k1 * R1 * cos(alpha) = Body2.X + k2 * R2 * cos(alpha) + L_ed * sqrt_L_mod ^ 2 * sin(alpha);
    Body1.Y + k1 * R1 * sin(alpha) = Body2.Y - k2 * R2 * sin(alpha) + L_ed * sqrt_L_mod ^ 2 * cos(alpha);
//L_actual=((Body1.X-Body2.X)^2 + (Body1.Y-Body2.Y)^2 - (k1 * R1 + k2 * R2)^2)^(1/2);
    Body1_out.Xp = Body1.X - k1 * R1 * cos(alpha);
    Body1_out.Yp = Body1.Y + k1 * R1 * sin(alpha);
    Body1_out.Fx = -T * sin(alpha);
    Body1_out.Fy = -T * cos(alpha);
    Body1_out.M = 0;
    Body2_out.Xp = Body2.X + k2 * R2 * cos(alpha);
    Body2_out.Yp = Body2.Y - k2 * R2 * sin(alpha);
    Body2_out.Fx = T * sin(alpha);
    Body2_out.Fy = T * cos(alpha);
    Body2_out.M = 0;
  end ThreadRR2D;

  model ThreadTest1
    parameter Modelica.SIunits.Acceleration g = 9.81;
    parameter Modelica.SIunits.Length R1 = 4;
    parameter Modelica.SIunits.Length R2 = 2;
    parameter Modelica.SIunits.Length L3 = 6;
    parameter Modelica.SIunits.Angle phi10 = 0;
    parameter Modelica.SIunits.Angle phi30 = -1;
    parameter Modelica.SIunits.Mass m1 = 2;
    parameter Modelica.SIunits.Mass m2 = 5;
    parameter Modelica.SIunits.Mass m3 = 2;
    parameter Modelica.SIunits.MomentOfInertia I1 = m1 * R1 ^ 2 / 2;
    parameter Modelica.SIunits.MomentOfInertia I2 = m2 * R2 ^ 2 / 2;
    parameter Modelica.SIunits.MomentOfInertia I3 = m3 * L3 ^ 2 / 12;
    parameter Modelica.SIunits.Length XO = -10;
    parameter Modelica.SIunits.Length YO = 0;
    parameter Modelica.SIunits.Length X20 = 0;
    parameter Modelica.SIunits.Length Y20 = 0;
    parameter Modelica.SIunits.Angle phiR = -0.7;
    parameter Modelica.SIunits.Length XR = X20 - R2 * sin(phiR);
    parameter Modelica.SIunits.Length YR = Y20 - R2 * cos(phiR);
    Modelica.SIunits.Energy E;
    ThreePortWheel Wheel1(phi0 = 1, R = R1, m = m1, I = I1);
    Seat2D Opora(Xp = XO, Yp = YO, Xb = 0, Yb = 0);
    Wheel2D Wheel2(phi0 = phi10, R = R2, m = m2, I = I2, X0 = X20, Y0 = Y20);
    Seat2D Opora2(Xp = X20, Yp = Y20, Xb = 0, Yb = 0);
    Rod2D Palka(phi0 = phi30, L = L3, m = m3, I = I3);
    Joint2D Sharnir(Xb1 = L3 / 2, Yb1 = 0, Xb2 = -R1, Yb2 = 0);
    // RollCircleOnLine Roll(Xp = XR, Yp = YR, Phip = phiR, R = R2);
    ThreadRR2D Thread(k1 = 1, k2 = 1, R1 = R1, R2 = R2);
    FreeEnd KonecKolesa;
  equation
    connect(Wheel1.Body, Opora.Body);
    connect(Wheel1.Body, Thread.Body1);
    connect(Wheel2.Body, Thread.Body2);
    connect(Palka.Body, Sharnir.Body1);
    connect(Wheel1.Body, Sharnir.Body2);
    connect(Wheel2.Body, Opora2.Body);
    connect(Wheel1.A, Opora.Body_out);
    connect(Wheel1.D, Sharnir.Body2_out);
    connect(Wheel1.B, Thread.Body1_out);
    connect(Wheel2.B, Thread.Body2_out);
//connect(Wheel2.A, Roll.Body_out);
    connect(Wheel2.A, Opora2.Body_out);
    connect(Palka.A, Sharnir.Body1_out);
    connect(Palka.B, KonecKolesa.Body_out);
    E = I1 * Wheel1.Omega ^ 2 / 2 + I2 * Wheel2.Omega ^ 2 / 2 + I3 * Palka.Omega ^ 2 / 2 + m3 * (Palka.Vx ^ 2 + Palka.Vy ^ 2) / 2 + m3 * g * Palka.Y;
    annotation(
      experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-06, Interval = 0.002));
  end ThreadTest1;

  model ElectricalEngine
    parameter Modelica.SIunits.Position Xp = 0;
    parameter Modelica.SIunits.Position Yp = 0;
    parameter Modelica.SIunits.Position Xb = 0;
    parameter Modelica.SIunits.Position Yb = 0;
    //
    parameter Modelica.SIunits.MomentOfForce M0 = 1;
    parameter Modelica.Mechanics.MultiBody.Frames.Orientation orientation = Modelica.Mechanics.MultiBody.Frames.axesRotations({1, 2, 3}, {0, 0, 0}, {0, 0, 0});
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape Seatshape(shapeType = "cylinder", length = 0.25, width = 0.1, height = 0.1, lengthDirection = {0, 0, 1}, widthDirection = {0, 1, 0}, color = {0, 0, 0}, specularCoefficient = 0.5, r = {Xp, Yp, 0}, R = orientation, r_shape = {0, 0, -0.1});
    KinematicInput Body;
    Modelica.SIunits.Force Rx;
    Modelica.SIunits.Force Ry;
    Modelica.SIunits.MomentOfForce M_dv;
    DynamicOutput Body_out;
  equation
    Xp = Body.X + Xb * cos(Body.Phi) - Yb * sin(Body.Phi);
    Yp = Body.Y + Xb * sin(Body.Phi) + Yb * cos(Body.Phi);
    M_dv = M0 * max({cos(Body.Phi), cos(Body.Phi + 2 * 3.14 / 3), cos(Body.Phi + 4 * 3.14 / 3)});
    Body_out.Xp = Xp;
    Body_out.Yp = Yp;
    Body_out.Fx = Rx;
    Body_out.Fy = Ry;
    Body_out.M = M_dv;
  end ElectricalEngine;

  model EEngineTest
    parameter Modelica.SIunits.Length L1 = 1;
    parameter Modelica.SIunits.Length L2 = 3;
    parameter Modelica.SIunits.AngularVelocity Omega1 = 2;
    parameter Modelica.SIunits.AngularVelocity Omega2 = 10;
    parameter Modelica.SIunits.Angle phi10 = 0;
    parameter Modelica.SIunits.Angle phi20 = 0;
    parameter Modelica.SIunits.Position Xp0 = L1 * cos(phi10) + L2 * cos(phi20);
    parameter Modelica.SIunits.Position Yp0 = L1 * sin(phi10) + L2 * sin(phi20);
    parameter Modelica.SIunits.Mass m1 = 1;
    parameter Modelica.SIunits.Mass m2 = 3;
    parameter Modelica.SIunits.MomentOfInertia I1 = m1 * L1 ^ 2 / 12;
    parameter Modelica.SIunits.MomentOfInertia I2 = m2 * L2 ^ 2 / 12;
    parameter Modelica.SIunits.MomentOfForce M_dv = 50;
    Rod2D Palka1(phi0 = phi10, L = L1, m = m1, I = I1);
    ElectricalEngine Engine(Xp = 0, Yp = 0, Xb = -L1 / 2, Yb = 0, M0 = M_dv);
    Rod2D Palka2(phi0 = phi20, L = L2, m = m2, I = I2);
    Joint2D Sharnir(Xb1 = L1 / 2, Yb1 = 0, Xb2 = -L2 / 2, Yb2 = 0);
    Slider2D Polzun(Xb = L2 / 2, Yb = 0, Xp = Xp0, Yp = Yp0, Phip = 0.5);
  equation
    connect(Palka1.Body, Engine.Body);
    connect(Palka1.Body, Sharnir.Body1);
    connect(Palka2.Body, Sharnir.Body2);
    connect(Palka2.Body, Polzun.Body);
    connect(Palka1.A, Engine.Body_out);
    connect(Palka1.B, Sharnir.Body1_out);
    connect(Palka2.A, Sharnir.Body2_out);
    connect(Palka2.B, Polzun.Body_out);
    annotation(
      experiment(StartTime = 0, StopTime = 20, Tolerance = 1e-06, Interval = 0.0004));
  end EEngineTest;

  model PistonEngine
    // parametry pryamoi v prostranstve
    parameter Modelica.SIunits.Position Xp = 0;
    parameter Modelica.SIunits.Position Yp = 0;
    parameter Modelica.SIunits.Angle Phip = 0;
    // parametry polzuna v tele
    parameter Modelica.SIunits.Position Xb = 0;
    parameter Modelica.SIunits.Position Yb = 0;
    //Force
    parameter Modelica.SIunits.Force F0 = 1;
    // animatsiya
    parameter Modelica.SIunits.Length BoxLength = 0.25;
    parameter Modelica.Mechanics.MultiBody.Frames.Orientation orientation = Modelica.Mechanics.MultiBody.Frames.axesRotations({1, 2, 3}, {0, 0, 0}, {0, 0, 0});
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape SliderJshape(shapeType = "cylinder", length = 0.25, width = 0.1, height = 0.1, lengthDirection = {0, 0, 1}, widthDirection = {0, 1, 0}, color = {0, 0, 0}, specularCoefficient = 0.5, r = {Xp + S * cos(Phip), Yp + S * sin(Phip), 0}, R = orientation, r_shape = {0, 0, 0});
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape SliderBshape(shapeType = "box", length = BoxLength, width = 0.2, height = 0.2, lengthDirection = {cos(Phip), sin(Phip), 0}, widthDirection = {0, 0, 1}, color = {0, 0, 160}, specularCoefficient = 0.5, r = {Xp + (S - BoxLength / 2) * cos(Phip), Yp + (S - BoxLength / 2) * sin(Phip), 0}, R = orientation, r_shape = {0, 0, 0});
    // polozhenie polzuna na pryamoi
    Modelica.SIunits.Length S;
    KinematicInput Body;
    DynamicOutput Body_out;
    Modelica.SIunits.Force N;
    Modelica.SIunits.Force F_dv;
  equation
    Xp + S * cos(Phip) = Body.X + Xb * cos(Body.Phi) - Yb * sin(Body.Phi);
    Yp + S * sin(Phip) = Body.Y + Xb * sin(Body.Phi) + Yb * cos(Body.Phi);
    if der(S) > 0.01 then
      F_dv = F0;
    elseif der(S) < (-0.01) then
      F_dv = -F0;
    else
      F_dv = 0;
    end if;
    Body_out.Xp = Xp + S * cos(Phip);
    Body_out.Yp = Yp + S * sin(Phip);
    Body_out.Fx = (-N * sin(Phip)) + F_dv * cos(Phip);
    Body_out.Fy = N * cos(Phip) + F_dv * sin(Phip);
    Body_out.M = 0;
  end PistonEngine;

  model PEngineTest
    parameter Modelica.SIunits.Length L1 = 1;
    parameter Modelica.SIunits.Length L2 = 3;
    parameter Modelica.SIunits.AngularVelocity Omega1 = 2;
    parameter Modelica.SIunits.AngularVelocity Omega2 = 10;
    parameter Modelica.SIunits.Angle phi10 = 0;
    parameter Modelica.SIunits.Angle phi20 = 0;
    parameter Modelica.SIunits.Position Xp0 = L1 * cos(phi10) + L2 * cos(phi20);
    parameter Modelica.SIunits.Position Yp0 = L1 * sin(phi10) + L2 * sin(phi20);
    parameter Modelica.SIunits.Mass m1 = 1;
    parameter Modelica.SIunits.Mass m2 = 3;
    parameter Modelica.SIunits.MomentOfInertia I1 = m1 * L1 ^ 2 / 12;
    parameter Modelica.SIunits.MomentOfInertia I2 = m2 * L2 ^ 2 / 12;
    parameter Modelica.SIunits.Force F0 = 3;
    Rod2D Palka1(phi0 = phi10, L = L1, m = m1, I = I1);
    Seat2D Opora(Xp = 0, Yp = 0, Xb = -L1 / 2, Yb = 0);
    Rod2D Palka2(phi0 = phi20, L = L2, m = m2, I = I2);
    Joint2D Sharnir(Xb1 = L1 / 2, Yb1 = 0, Xb2 = -L2 / 2, Yb2 = 0);
    PistonEngine Engine(Xb = L2 / 2, Yb = 0, Xp = Xp0, Yp = Yp0, Phip = 0, F0 = F0);
  equation
    connect(Palka1.Body, Opora.Body);
    connect(Palka1.Body, Sharnir.Body1);
    connect(Palka2.Body, Sharnir.Body2);
    connect(Palka2.Body, Engine.Body);
    connect(Palka1.A, Opora.Body_out);
    connect(Palka1.B, Sharnir.Body1_out);
    connect(Palka2.A, Sharnir.Body2_out);
    connect(Palka2.B, Engine.Body_out);
    annotation(
      experiment(StartTime = 0, StopTime = 100, Tolerance = 1e-6, Interval = 0.002));
  end PEngineTest;

  model ThreadTest2
    parameter Modelica.SIunits.Acceleration g = 9.81;
    parameter Modelica.SIunits.Length R1 = 4;
    parameter Modelica.SIunits.Length R2 = 2;
    parameter Modelica.SIunits.Length L3 = 6;
    parameter Modelica.SIunits.Angle phi10 = 0;
    parameter Modelica.SIunits.Angle phi30 = -1;
    parameter Modelica.SIunits.Mass m1 = 2;
    parameter Modelica.SIunits.Mass m2 = 5;
    parameter Modelica.SIunits.Mass m3 = 2;
    parameter Modelica.SIunits.MomentOfInertia I1 = m1 * R1 ^ 2 / 2;
    parameter Modelica.SIunits.MomentOfInertia I2 = m2 * R2 ^ 2 / 2;
    parameter Modelica.SIunits.MomentOfInertia I3 = m3 * L3 ^ 2 / 12;
    parameter Modelica.SIunits.Length XO = -10;
    parameter Modelica.SIunits.Length YO = 0;
    parameter Modelica.SIunits.Length X20 = XO + R1 + R2;
    parameter Modelica.SIunits.Length Y20 = YO - 10;
    parameter Modelica.SIunits.Angle phiR = -0.7;
    parameter Modelica.SIunits.Length XR = X20 - R2 * sin(phiR);
    parameter Modelica.SIunits.Length YR = Y20 - R2 * cos(phiR);
    parameter Real k1 = -1;
    parameter Real k2 = -1;
    parameter Modelica.SIunits.Length L_0 = 10;
    //(XO^2 + YO^2 - (k1 * R1 + k2 * R2)^2)^(1/2);
    Modelica.SIunits.Energy E;
    ThreePortWheel Wheel1(phi0 = phi10, R = R1, m = m1, I = I1);
    Seat2D Opora(Xp = XO, Yp = YO, Xb = 0, Yb = 0);
    Wheel2D Wheel2(R = R2, m = m2, I = I2, X0 = X20, Y0 = Y20);
    //Seat2D Opora2(Xp = X20, Yp = Y20, Xb = 0, Yb = 0);
    Rod2D Palka(phi0 = phi30, L = L3, m = m3, I = I3);
    Joint2D Sharnir(Xb1 = L3 / 2, Yb1 = 0, Xb2 = -R1, Yb2 = 0);
    // RollCircleOnLine Roll(Xp = XR, Yp = YR, Phip = phiR, R = R2);
    ThreadRR2D Thread(k1 = -1, k2 = -1, R1 = R1, R2 = R2, L_0 = L_0);
    FreeEnd KonecKolesa;
    FreeEnd KonecKolesa2;
  equation
    connect(Wheel1.Body, Opora.Body);
    connect(Wheel1.Body, Thread.Body1);
    connect(Wheel2.Body, Thread.Body2);
    connect(Palka.Body, Sharnir.Body1);
    connect(Wheel1.Body, Sharnir.Body2);
//  connect(Wheel2.Body, KonecKolesa2.Body);
    connect(Wheel1.A, Opora.Body_out);
    connect(Wheel1.D, Sharnir.Body2_out);
    connect(Wheel1.B, Thread.Body1_out);
    connect(Wheel2.B, Thread.Body2_out);
//connect(Wheel2.A, Roll.Body_out);
    connect(Wheel2.A, KonecKolesa2.Body_out);
    connect(Palka.A, Sharnir.Body1_out);
    connect(Palka.B, KonecKolesa.Body_out);
    E = I1 * Wheel1.Omega ^ 2 / 2 + I2 * Wheel2.Omega ^ 2 / 2 + I3 * Palka.Omega ^ 2 / 2 + m3 * (Palka.Vx ^ 2 + Palka.Vy ^ 2) / 2 + m3 * g * Palka.Y;
    annotation(
      experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-06, Interval = 0.0002));
  end ThreadTest2;

  model Systemtest
    parameter Modelica.SIunits.Acceleration g = 9.81;
    parameter Modelica.SIunits.Length R1 = 4;
    parameter Modelica.SIunits.Length R2 = 2;
    parameter Modelica.SIunits.Length L3 = 10;
    parameter Modelica.SIunits.Angle phi10 = 3;
    parameter Modelica.SIunits.Angle phi30 = -1.57;
    parameter Modelica.SIunits.Mass m1 = 2;
    parameter Modelica.SIunits.Mass m2 = 5;
    parameter Modelica.SIunits.Mass m3 = 2;
    parameter Modelica.SIunits.MomentOfInertia I1 = m1 * R1 ^ 2 / 2;
    parameter Modelica.SIunits.MomentOfInertia I2 = m2 * R2 ^ 2 / 2;
    parameter Modelica.SIunits.MomentOfInertia I3 = m3 * L3 ^ 2 / 12;
    parameter Modelica.SIunits.Length XO = -10;
    parameter Modelica.SIunits.Length YO = 0;
    parameter Modelica.SIunits.Length X20 = 0;
    parameter Modelica.SIunits.Length Y20 = 0;
    parameter Modelica.SIunits.Angle phiR = -0.7;
    parameter Modelica.SIunits.Length XR = X20 - R2 * sin(phiR);
    parameter Modelica.SIunits.Length YR = Y20 - R2 * cos(phiR);
    parameter Modelica.SIunits.Force F0 = 10;
    Modelica.SIunits.Energy E;
    ThreePortWheel Wheel1(phi0 = phi10, R = R1, m = m1, I = I1);
    Seat2D Opora(Xp = XO, Yp = YO, Xb = 0, Yb = 0);
    Wheel2D Wheel2(R = R2, m = m2, I = I2, X0 = X20, Y0 = Y20);
    Seat2D Opora2(Xp = X20, Yp = Y20, Xb = 0, Yb = 0);
    Rod2D Palka(phi0 = phi30, L = L3, m = m3, I = I3);
    Joint2D Sharnir(Xb1 = L3 / 2, Yb1 = 0, Xb2 = -R1, Yb2 = 0);
    // RollCircleOnLine Roll(Xp = XR, Yp = YR, Phip = phiR, R = R2);
    ThreadRR2D Thread(k1 = -1, k2 = -1, R1 = R1, R2 = R2);
    PistonEngine Engine(Xb = -L3 / 2, Yb = 0, Xp = XO, Yp = YO, Phip = 1.57, F0 = F0);
    //FreeEnd KonecKolesa;
  equation
    connect(Wheel1.Body, Opora.Body);
    connect(Wheel1.Body, Thread.Body1);
    connect(Wheel2.Body, Thread.Body2);
    connect(Palka.Body, Sharnir.Body1);
    connect(Wheel1.Body, Sharnir.Body2);
    connect(Wheel2.Body, Opora2.Body);
    connect(Wheel1.A, Opora.Body_out);
    connect(Wheel1.D, Sharnir.Body2_out);
    connect(Wheel1.B, Thread.Body1_out);
    connect(Wheel2.B, Thread.Body2_out);
//connect(Wheel2.A, Roll.Body_out);
    connect(Wheel2.A, Opora2.Body_out);
    connect(Palka.A, Sharnir.Body1_out);
    connect(Palka.B, Engine.Body_out);
    connect(Palka.Body, Engine.Body);
    E = I1 * Wheel1.Omega ^ 2 / 2 + I2 * Wheel2.Omega ^ 2 / 2 + I3 * Palka.Omega ^ 2 / 2 + m3 * (Palka.Vx ^ 2 + Palka.Vy ^ 2) / 2 + m3 * g * Palka.Y;
    annotation(
      experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-06, Interval = 0.002));
  end Systemtest;

  model SlideOnPlane2D
    // parametry pryamoi v prostranstve
    parameter Modelica.SIunits.Position Xp = 0;
    parameter Modelica.SIunits.Position Yp = 0;
    parameter Modelica.SIunits.Angle Phip = 0;
    // parametry polzuna v tele
    parameter Modelica.SIunits.Position Xb = 0;
    parameter Modelica.SIunits.Position Yb = 0;
    parameter Modelica.SIunits.Angle Phib = 0;
    // animatsiya
    parameter Modelica.SIunits.Length L = 4;
    parameter Modelica.Mechanics.MultiBody.Frames.Orientation orientation = Modelica.Mechanics.MultiBody.Frames.axesRotations({1, 2, 3}, {0, 0, 0}, {0, 0, 0});
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape Planeshape(shapeType = "box", length = L, width = 0.2, height = 0.05, lengthDirection = {cos(Phip), sin(Phip), 0}, widthDirection = {0, 0, 1}, color = {0, 0, 160}, specularCoefficient = 0.5, r = {Xp + 0.025 * sin(Phip), Yp - 0.025 * cos(Phip), 0}, R = orientation, r_shape = {0, 0, 0});
    // polozhenie polzuna na pryamoi
    Modelica.SIunits.Length S;
    KinematicInput Body;
    DynamicOutput Body_out;
    Modelica.SIunits.Force N;
    Modelica.SIunits.MomentOfForce M;
  equation
    Xp + S * cos(Phip) = Body.X + Xb * cos(Body.Phi) - Yb * sin(Body.Phi);
    Yp + S * sin(Phip) = Body.Y + Xb * sin(Body.Phi) + Yb * cos(Body.Phi);
    Phip = Body.Phi + Phib;
    Body_out.Xp = Xp + S * cos(Phip);
    Body_out.Yp = Yp + S * sin(Phip);
    Body_out.Fx = -N * sin(Phip);
    Body_out.Fy = N * cos(Phip);
    Body_out.M = M;
  end SlideOnPlane2D;

  model Box2D
    extends DynamicSandBox.TwoPortBody2D;
    parameter Modelica.SIunits.Length a = 1;
    parameter Modelica.SIunits.Length b = 1;
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape Bodyshape(shapeType = "box", length = a, width = 0.05, height = b, lengthDirection = {cos(Phi), sin(Phi), 0}, widthDirection = {0, 0, 1}, color = Color, specularCoefficient = 0.5, r = {X - a / 2 * cos(Phi), Y - a / 2 * sin(Phi), 0}, R = orientation, r_shape = {0, 0, 0});
  end Box2D;

  model SlideBoxTest
    parameter Modelica.SIunits.Length a = 1;
    parameter Modelica.SIunits.Length b = 0.5;
    parameter Modelica.SIunits.Angle phi10 = 0;
    parameter Modelica.SIunits.Mass m1 = 2;
    parameter Modelica.SIunits.MomentOfInertia I1 = m1 * a ^ 2 / 12;
    parameter Modelica.SIunits.Angle phip = -0.1;
    Box2D Box(a = a, b = b, m = m1, I = I1);
    SlideOnPlane2D Plane(Xp = 0, Yp = 0, Phip = phip, Xb = 0, Yb = -b / 2, Phib = 0);
    FreeEnd KonecPalki;
  equation
    connect(Box.Body, Plane.Body);
    connect(Box.A, Plane.Body_out);
    connect(Box.B, KonecPalki.Body_out);
    annotation(
      experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-06, Interval = 0.0002));
  end SlideBoxTest;

  model RealJoint2D
    parameter Modelica.SIunits.Position Xb1 = 0;
    parameter Modelica.SIunits.Position Yb1 = 0;
    parameter Modelica.SIunits.Position Xb2 = 0;
    parameter Modelica.SIunits.Position Yb2 = 0;
    parameter Real k = 0;
    parameter Modelica.SIunits.MomentOfForce M_ed = 1;
    parameter Modelica.SIunits.AngularVelocity Omega_ed = 1;
    parameter Modelica.Mechanics.MultiBody.Frames.Orientation orientation = Modelica.Mechanics.MultiBody.Frames.axesRotations({1, 2, 3}, {0, 0, 0}, {0, 0, 0});
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape Jointshape(shapeType = "cylinder", length = 0.25, width = 0.1, height = 0.1, lengthDirection = {0, 0, 1}, widthDirection = {0, 1, 0}, color = {0, 0, 0}, specularCoefficient = 0.5, r = {XSh, YSh, 0}, R = orientation, r_shape = {0, 0, -0.1});
    Modelica.SIunits.Position XSh;
    Modelica.SIunits.Position YSh;
    KinematicInput Body1;
    KinematicInput Body2;
    Modelica.SIunits.Force Rx;
    Modelica.SIunits.Force Ry;
    Modelica.SIunits.MomentOfForce M_vtr;
    DynamicOutput Body1_out;
    DynamicOutput Body2_out;
  equation
    XSh = Body1.X + Xb1 * cos(Body1.Phi) - Yb1 * sin(Body1.Phi);
    YSh = Body1.Y + Xb1 * sin(Body1.Phi) + Yb1 * cos(Body1.Phi);
    XSh = Body2.X + Xb2 * cos(Body2.Phi) - Yb2 * sin(Body2.Phi);
    YSh = Body2.Y + Xb2 * sin(Body2.Phi) + Yb2 * cos(Body2.Phi);
    M_vtr = k * der(Body1.Phi - Body2.Phi);
    Body1_out.Xp = XSh;
    Body1_out.Yp = YSh;
    Body1_out.Fx = Rx;
    Body1_out.Fy = Ry;
    Body1_out.M = M_vtr;
    Body2_out.Xp = XSh;
    Body2_out.Yp = YSh;
    Body2_out.Fx = -Rx;
    Body2_out.Fy = -Ry;
    Body2_out.M = -M_vtr;
  end RealJoint2D;

  model RealJointTest
    parameter Modelica.SIunits.Length L1 = 4;
    parameter Modelica.SIunits.Length L2 = 2;
    parameter Modelica.SIunits.Angle phi10 = 1;
    parameter Modelica.SIunits.Angle phi20 = -1;
    parameter Modelica.SIunits.Mass m1 = 2;
    parameter Modelica.SIunits.Mass m2 = 5;
    parameter Modelica.SIunits.MomentOfInertia I1 = m1 * L1 ^ 2 / 12;
    parameter Modelica.SIunits.MomentOfInertia I2 = m2 * L2 ^ 2 / 12;
    parameter Real k = 100;
    Rod2D Palka1(phi0 = phi10, L = L1, m = m1, I = I1);
    RealSeat2D RealOpora(Xp = 1, Yp = 3, Xb = -L1 / 2, Yb = 0, k = k);
    Rod2D Palka2(phi0 = phi20, L = L2, m = m2, I = I2);
    RealJoint2D RealSharnir(Xb1 = L1 / 2, Yb1 = 0, Xb2 = -L2 / 2, Yb2 = 0, k = k);
    FreeEnd KonecPalki;
  equation
    connect(Palka1.Body, RealOpora.Body);
    connect(Palka1.Body, RealSharnir.Body1);
    connect(Palka2.Body, RealSharnir.Body2);
    connect(Palka1.A, RealOpora.Body_out);
    connect(Palka1.B, RealSharnir.Body2_out);
    connect(Palka2.B, RealSharnir.Body1_out);
    connect(Palka2.A, KonecPalki.Body_out);
    annotation(
      experiment(StartTime = 0, StopTime = 30, Tolerance = 1e-06, Interval = 0.0006));
  end RealJointTest;

  model RealSeat2D
    parameter Modelica.SIunits.Position Xp = 0;
    parameter Modelica.SIunits.Position Yp = 0;
    parameter Modelica.SIunits.Position Xb = 0;
    parameter Modelica.SIunits.Position Yb = 0;
    parameter Real k = 0;
    parameter Modelica.SIunits.MomentOfForce M_ed = 1;
    parameter Modelica.SIunits.AngularVelocity Omega_ed = 1;
    parameter Modelica.Mechanics.MultiBody.Frames.Orientation orientation = Modelica.Mechanics.MultiBody.Frames.axesRotations({1, 2, 3}, {0, 0, 0}, {0, 0, 0});
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape Seatshape(shapeType = "cylinder", length = 0.25, width = 0.1, height = 0.1, lengthDirection = {0, 0, 1}, widthDirection = {0, 1, 0}, color = {0, 0, 0}, specularCoefficient = 0.5, r = {Xp, Yp, 0}, R = orientation, r_shape = {0, 0, -0.1});
    KinematicInput Body;
    Modelica.SIunits.Force Rx;
    Modelica.SIunits.Force Ry;
    Modelica.SIunits.MomentOfForce M_vtr;
    DynamicOutput Body_out;
  equation
    Xp = Body.X + Xb * cos(Body.Phi) - Yb * sin(Body.Phi);
    Yp = Body.Y + Xb * sin(Body.Phi) + Yb * cos(Body.Phi);
    M_vtr = k * der(Body.Phi);
    Body_out.Xp = Xp;
    Body_out.Yp = Yp;
    Body_out.Fx = Rx;
    Body_out.Fy = Ry;
    Body_out.M = -M_vtr;
  end RealSeat2D;

  model ThreadRP2D
    parameter Modelica.SIunits.Length L0 = 0;
    parameter Modelica.SIunits.Length R1 = 1;
    parameter Real k1 = 1;
    parameter Modelica.SIunits.Position Xb2 = 0;
    parameter Modelica.SIunits.Position Yb2 = 0;
    parameter Modelica.Mechanics.MultiBody.Frames.Orientation orientation = Modelica.Mechanics.MultiBody.Frames.axesRotations({1, 2, 3}, {0, 0, 0}, {0, 0, 0});
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape Threadshape(shapeType = "cylinder", length = L_ed * sqrt_L_mod ^ 2, width = 0.05, height = 0.05, lengthDirection = {sin(alpha), cos(alpha), 0}, widthDirection = {0, 0, 1}, color = {0, 0, 0}, specularCoefficient = 0.5, r = {Body2.X + Xb2 * cos(Body2.Phi) - Yb2 * sin(Body2.Phi), Body2.Y + Xb2 * sin(Body2.Phi) + Yb2 * cos(Body2.Phi), 0}, R = orientation, r_shape = {0, 0, -0.1});
    parameter Modelica.SIunits.Length L_ed = 1;
    parameter Modelica.SIunits.Length L_0 = 0;
    Real sqrt_L_mod;
    Modelica.SIunits.Angle alpha;
    KinematicInput Body1;
    KinematicInput Body2;
    Modelica.SIunits.Force T;
    DynamicOutput Body1_out;
    DynamicOutput Body2_out;
  equation
    L_ed * sqrt_L_mod ^ 2 = L_0 + k1 * R1 * (Body1.Phi + alpha);
    Body1.X - k1 * R1 * cos(alpha) = Body2.X + Xb2 * cos(Body2.Phi) - Yb2 * sin(Body2.Phi) + L_ed * sqrt_L_mod ^ 2 * sin(alpha);
    Body1.Y + k1 * R1 * sin(alpha) = Body2.Y + Xb2 * sin(Body2.Phi) + Yb2 * cos(Body2.Phi) + L_ed * sqrt_L_mod ^ 2 * cos(alpha);
//L_actual=((Body1.X-Body2.X)^2 + (Body1.Y-Body2.Y)^2 - (k1 * R1 + k2 * R2)^2)^(1/2);
    Body1_out.Xp = Body1.X - k1 * R1 * cos(alpha);
    Body1_out.Yp = Body1.Y + k1 * R1 * sin(alpha);
    Body1_out.Fx = -T * sin(alpha);
    Body1_out.Fy = -T * cos(alpha);
    Body1_out.M = 0;
    Body2_out.Xp = Body2.X + Xb2 * cos(Body2.Phi) - Yb2 * sin(Body2.Phi);
    Body2_out.Yp = Body2.Y + Xb2 * sin(Body2.Phi) + Yb2 * cos(Body2.Phi);
    Body2_out.Fx = T * sin(alpha);
    Body2_out.Fy = T * cos(alpha);
    Body2_out.M = 0;
  end ThreadRP2D;

  model ThreadRPTest
    parameter Modelica.SIunits.Acceleration g = 9.81;
    parameter Modelica.SIunits.Length R2 = 2;
    parameter Modelica.SIunits.Length L3 = 6;
    parameter Modelica.SIunits.Angle phi20 = 0;
    parameter Modelica.SIunits.Angle phi30 = 0;
    parameter Modelica.SIunits.Mass m2 = 5;
    parameter Modelica.SIunits.Mass m3 = 2;
    parameter Modelica.SIunits.MomentOfInertia I2 = m2 * R2 ^ 2 / 2;
    parameter Modelica.SIunits.MomentOfInertia I3 = m3 * L3 ^ 2 / 12;
    parameter Modelica.SIunits.Length XP = -10;
    parameter Modelica.SIunits.Length YP = 0;
    parameter Modelica.SIunits.Length X20 = 0;
    parameter Modelica.SIunits.Length Y20 = 0;
    parameter Modelica.SIunits.Length L0 = ((XP + L3 / 3) ^ 2 - R2 ^ 2) ^ 0.5;
    // Modelica.SIunits.Energy E;
    Wheel2D Wheel2(phi0 = phi20, R = R2, m = m2, I = I2);
    Seat2D Opora2(Xp = X20, Yp = Y20, Xb = 0, Yb = 0);
    Rod2D Palka(phi0 = phi30, L = L3, m = m3, I = I3, X0 = XP, Y0 = YP);
    //Joint2D Sharnir(Xb1 = L3 / 2, Yb1 = 0, Xb2 = -R1, Yb2 = 0);
    // RollCircleOnLine Roll(Xp = XR, Yp = YR, Phip = phiR, R = R2);
    ThreadRP2D Thread(k1 = -1, R1 = R2, Xb2 = -L3 / 2, Yb2 = 0, L_0 = L0);
    FreeEnd KonecKolesa;
  equation
// connect(Wheel1.Body, Opora.Body);
// connect(Wheel1.Body, Thread.Body1);
    connect(Wheel2.Body, Thread.Body1);
    connect(Palka.Body, Thread.Body2);
// connect(Wheel1.Body, Sharnir.Body2);
    connect(Wheel2.Body, Opora2.Body);
// connect(Wheel1.A, Opora.Body_out);
// connect(Wheel1.D, Sharnir.Body2_out);
// connect(Wheel1.B, Thread.Body1_out);
    connect(Wheel2.B, Thread.Body1_out);
//connect(Wheel2.A, Roll.Body_out);
    connect(Wheel2.A, Opora2.Body_out);
    connect(Palka.A, Thread.Body2_out);
    connect(Palka.B, KonecKolesa.Body_out);
//  E = I1 * Wheel1.Omega ^ 2 / 2 + I2 * Wheel2.Omega ^ 2 / 2 + I3 * Palka.Omega ^ 2 / 2 + m3 * (Palka.Vx ^ 2 + Palka.Vy ^ 2) / 2 + m3 * g * Palka.Y;
  end ThreadRPTest;

  model CheatedRCCB2D
    parameter Modelica.SIunits.Angle phi0 = 1;
    parameter Real R1 = 1;
    parameter Real R2 = 1;
    KinematicInput Body1;
    KinematicInput Body2;
    Modelica.SIunits.Force Ftr;
    DynamicOutput Body1_out;
    DynamicOutput Body2_out;
    Real CosAlp;
    Real SinAlp;
  equation
    R1 * der(Body1.Phi) = -R2 * der(Body2.Phi);
    CosAlp = (Body1.X - Body2.X) / ((Body1.X - Body2.X) ^ 2 + (Body1.Y - Body2.Y) ^ 2) ^ 0.5;
    SinAlp = (Body1.Y - Body2.Y) / ((Body1.X - Body2.X) ^ 2 + (Body1.Y - Body2.Y) ^ 2) ^ 0.5;
    Body1_out.Xp = Body1.X - R1 * CosAlp;
    Body1_out.Yp = Body1.Y - R1 * SinAlp;
    Body1_out.Fx = -Ftr * SinAlp;
    Body1_out.Fy = Ftr * CosAlp;
    Body1_out.M = 0;
    Body2_out.Xp = Body1.X - R1 * CosAlp;
    Body2_out.Yp = Body1.Y - R1 * SinAlp;
    Body2_out.Fx = Ftr * SinAlp;
    Body2_out.Fy = -Ftr * CosAlp;
    Body2_out.M = 0;
  end CheatedRCCB2D;

  model RollRollTest
    parameter Modelica.SIunits.Length R1 = 1;
    parameter Modelica.SIunits.Length R21 = 4;
    parameter Modelica.SIunits.Length R22 = 1;
    parameter Modelica.SIunits.Length R3 = 8;
    parameter Modelica.SIunits.Angle phi10 = 0;
    // parameter Modelica.SIunits.Angle phi20 = 0;
    //parameter Modelica.SIunits.Angle phi30 = 0;
    //parameter Modelica.SIunits.Angle phip = 0.7;
    //parameter Modelica.SIunits.AngularVelocity Omega1 = 1;
    parameter Modelica.SIunits.Position Xo1 = 0;
    parameter Modelica.SIunits.Position Yo1 = 0;
    parameter Modelica.SIunits.Position Xo2 = R1 + R21;
    parameter Modelica.SIunits.Position Yo2 = 0;
    parameter Modelica.SIunits.Position Xo3 = R1 + R21;
    parameter Modelica.SIunits.Position Yo3 = R22 + R3;
    parameter Modelica.SIunits.Mass m1 = 1;
    parameter Modelica.SIunits.Mass m2 = 5;
    parameter Modelica.SIunits.Mass m3 = 5;
    parameter Modelica.SIunits.MomentOfInertia i1 = m1 * R1 ^ 2 / 2;
    parameter Modelica.SIunits.MomentOfInertia i2 = m2 * R21 ^ 2 / 2;
    parameter Modelica.SIunits.MomentOfInertia i3 = m3 * R3 ^ 2 / 2;
    parameter Modelica.SIunits.MomentOfForce M_dv = 50;
    parameter Real k1 = 10;
    parameter Real k2 = 20;
    ElectricalEngine Engine(Xp = Xo1, Yp = Yo1, Xb = 0, Yb = 0, M0 = M_dv);
    Wheel2D Wheel1(X0 = Xo1, Y0 = Yo1, phi0 = 0, R = R1, Color = {0, 255, 255}, m = m1, I = i1);
    RealSeat2D Support(Xp = Xo2, Yp = Yo2, Xb = 0, Yb = 0, k = k1);
    ThreePortStepWheel2D Wheel2(X0 = Xo2, Y0 = Yo2, phi0 = 0, R1 = R21, R2 = R22, m = m2, I = i2, Color = {255, 0, 255});
    CheatedRCCB2D roll(R1 = R1, R2 = R21);
    RealSeat2D Support2(Xp = Xo3, Yp = Yo3, Xb = 0, Yb = 0, k = k2);
    Wheel2D Wheel3(X0 = Xo3, Y0 = Yo3, phi0 = 0, R = R3, m = m3, I = i3);
    CheatedRCCB2D roll2(R1 = R22, R2 = R3);
  equation
    connect(Wheel1.Body, Engine.Body);
    connect(Wheel1.Body, roll.Body1);
    connect(Wheel2.Body, roll.Body2);
    connect(Wheel2.Body, Support.Body);
    connect(Wheel2.Body, roll2.Body1);
    connect(Wheel3.Body, roll2.Body2);
    connect(Wheel3.Body, Support2.Body);
    connect(Wheel1.A, Engine.Body_out);
    connect(Wheel1.B, roll.Body1_out);
    connect(Wheel2.A, roll.Body2_out);
    connect(Wheel2.B, Support.Body_out);
    connect(Wheel2.D, roll2.Body1_out);
    connect(Wheel3.A, roll2.Body2_out);
    connect(Wheel3.B, Support2.Body_out);
    annotation(
      experiment(StartTime = 0, StopTime = 100, Tolerance = 1e-06, Interval = 0.002));
  end RollRollTest;

  model ThreePortStepWheel2D
    extends DynamicSandBox.ThreePortBody2D;
    parameter Modelica.SIunits.Length R1 = 1;
    parameter Modelica.SIunits.Length R2 = 0.75;
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape Bodyshape1(shapeType = "cylinder", length = 0.05, width = 2 * R1, height = 2 * R1, lengthDirection = {0, 0, 1}, widthDirection = {cos(Phi), sin(Phi), 0}, color = Color, specularCoefficient = 0.5, r = {X, Y, 0}, R = orientation, r_shape = {0, 0, 0});
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape Bodyshape2(shapeType = "cylinder", length = 0.1, width = 2 * R2, height = 2 * R2, lengthDirection = {0, 0, 1}, widthDirection = {cos(Phi), sin(Phi), 0}, color = Color * 0.1, specularCoefficient = 0.5, r = {X, Y, 0}, R = orientation, r_shape = {0, 0, 0});
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape Bodyshape3(shapeType = "box", length = 0.15, width = R2, height = R2, lengthDirection = {0, 0, 1}, widthDirection = {cos(Phi), sin(Phi), 0}, color = Color * 0.5, specularCoefficient = 0.5, r = {X, Y, 0}, R = orientation, r_shape = {0, 0, 0});
  end ThreePortStepWheel2D;

  model CheatedThreadRRBPC2D
    parameter Modelica.SIunits.Angle phi0 = 1;
    parameter Modelica.SIunits.Length R1 = 1;
    parameter Modelica.SIunits.Length R2 = 1;
    parameter Modelica.SIunits.Angle alpha = 0;
    parameter Real k1 = 1;
    parameter Modelica.Mechanics.MultiBody.Frames.Orientation orientation = Modelica.Mechanics.MultiBody.Frames.axesRotations({1, 2, 3}, {0, 0, 0}, {0, 0, 0});
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape Threadshape(shapeType = "cylinder", length = L_actual, width = 0.05, height = 0.05, lengthDirection = {Cos_actual, Sin_actual, 0}, widthDirection = {0, 0, 1}, color = {0, 0, 0}, specularCoefficient = 0.5, r = {Body2.X, Body2.Y, 0}, R = orientation, r_shape = {0, 0, 0.1});
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape Pointshape(shapeType = "cylinder", length = 0.25, width = 0.1, height = 0.1, lengthDirection = {0, 0, 1}, widthDirection = {0, 1, 0}, color = {0, 0, 0}, specularCoefficient = 0.5, r = {Body2.X, Body2.Y, 0}, R = orientation, r_shape = {0, 0, -0.1});
    Modelica.SIunits.Length L_actual;
    Real Sin_actual;
    Real Cos_actual;
    KinematicInput Body1;
    //Based Disk
    KinematicInput Body2;
    //Rolling Disk
    Modelica.SIunits.Force T;
    DynamicOutput Body1_out;
    DynamicOutput Body2_out;
  equation
    -k1 * R1 * der(Body1.Phi) = R2 * der(Body2.Phi);
    Sin_actual = (Body1.Y + R1 * cos(alpha) - Body2.Y) / L_actual;
    Cos_actual = (Body1.X - R1 * sin(alpha) - Body2.X) / L_actual;
    L_actual = ((Body1.X - R1 * sin(alpha) - Body2.X) ^ 2 + (Body1.Y + R1 * cos(alpha) - Body2.Y) ^ 2) ^ 0.5;
    Body1_out.Xp = Body1.X - R1 * sin(alpha);
    Body1_out.Yp = Body1.Y + R1 * cos(alpha);
    Body1_out.Fx = -T * cos(alpha);
    Body1_out.Fy = -T * sin(alpha);
    Body1_out.M = 0;
    Body2_out.Xp = Body2.X;
    Body2_out.Yp = Body2.Y;
    Body2_out.Fx = T * cos(alpha);
    Body2_out.Fy = T * sin(alpha);
    Body2_out.M = 0;
  end CheatedThreadRRBPC2D;

  model CheatedThreadTest1
    parameter Modelica.SIunits.Length R1 = 1;
    parameter Modelica.SIunits.Length R2 = 5;
    parameter Modelica.SIunits.Length L0 = 10;
    parameter Modelica.SIunits.Angle phi10 = 0;
    parameter Modelica.SIunits.Angle phi20 = 0;
    parameter Modelica.SIunits.Angle phi30 = 0;
    parameter Modelica.SIunits.Angle alpha = 0.5;
    parameter Modelica.SIunits.Position XO = 0;
    parameter Modelica.SIunits.Position YO = 0;
    parameter Modelica.SIunits.Position Xp0 = XO - k1 * R1 * sin(alpha) + R2 * sin(alpha) - L0 * cos(alpha);
    parameter Modelica.SIunits.Position Yp0 = YO + k1 * R1 * cos(alpha) - R2 * cos(alpha) - L0 * sin(alpha);
    parameter Modelica.SIunits.Mass m1 = 1;
    parameter Modelica.SIunits.Mass m2 = 3;
    parameter Modelica.SIunits.Mass m3 = 5;
    parameter Modelica.SIunits.MomentOfInertia I1 = m1 * R1 ^ 2 / 2;
    parameter Modelica.SIunits.MomentOfInertia I2 = m2 * R2 ^ 2 / 2;
    parameter Real k1 = 1;
    Seat2D Opora(Xp = XO, Yp = YO, Xb = 0, Yb = 0);
    Wheel2D Koleso(X0 = XO, Y0 = YO, R = R1, m = m1, I = I1);
    CheatedThreadRRBPC2D Nitka(R1 = R1, R2 = R2, alpha = alpha, k1 = k1);
    Wheel2D Koleso2(X0 = Xp0, Y0 = Yp0, R = R2, m = m2, I = I2);
    RollCircleOnLine Roll(Xp = Xp0, Yp = Yp0, Phip = alpha, R = R2);
  equation
    connect(Koleso.Body, Opora.Body);
    connect(Koleso.Body, Nitka.Body1);
    connect(Koleso2.Body, Nitka.Body2);
    connect(Koleso2.Body, Roll.Body);
    connect(Koleso.A, Opora.Body_out);
    connect(Koleso.B, Nitka.Body1_out);
    connect(Koleso2.A, Roll.Body_out);
    connect(Koleso2.B, Nitka.Body2_out);
    annotation(
      experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-06, Interval = 0.0002));
  end CheatedThreadTest1;

  model SystemNumber29
    parameter Modelica.SIunits.Length R1 = 2;
    parameter Modelica.SIunits.Length R21 = 3;
    parameter Modelica.SIunits.Length R22 = 2;
    parameter Modelica.SIunits.Length R3 = 5;
    parameter Modelica.SIunits.Length L4 = 9;
    parameter Modelica.SIunits.Angle phi10 = 0;
    // parameter Modelica.SIunits.Angle phi20 = 0;
    //parameter Modelica.SIunits.Angle phi30 = 0;
    //parameter Modelica.SIunits.Angle phip = 0.7;
    //parameter Modelica.SIunits.AngularVelocity Omega1 = 1;
    parameter Modelica.SIunits.Position Xo1 = 5;
    parameter Modelica.SIunits.Position Yo1 = 0;
    parameter Modelica.SIunits.Position Xo2 = Xo1 - R1 - R21;
    parameter Modelica.SIunits.Position Yo2 = Yo1;
    parameter Modelica.SIunits.Position Xo3 = Xo2 - R22 - R3 - 2;
    parameter Modelica.SIunits.Position Yo3 = Yo2 + R22 + R3;
    parameter Modelica.SIunits.Mass m1 = 1;
    parameter Modelica.SIunits.Mass m2 = 5;
    parameter Modelica.SIunits.Mass m3 = 5;
    parameter Modelica.SIunits.Mass m4 = 2;
    parameter Modelica.SIunits.MomentOfInertia i1 = m1 * R1 ^ 2 / 2;
    parameter Modelica.SIunits.MomentOfInertia i2 = m2 * R21 ^ 2 / 2;
    parameter Modelica.SIunits.MomentOfInertia i3 = m3 * R3 ^ 2 / 2;
    parameter Modelica.SIunits.MomentOfInertia i4 = m4 * L4 ^ 2 / 12;
    parameter Modelica.SIunits.MomentOfForce M_dv = 50;
    parameter Real k1 = 10;
    parameter Real k2 = 20;
    ElectricalEngine Engine(Xp = Xo1, Yp = Yo1, Xb = 0, Yb = 0, M0 = M_dv);
    Wheel2D Wheel1(X0 = Xo1, Y0 = Yo1, phi0 = 0, R = R1, Color = {0, 255, 255}, m = m1, I = i1);
    RealSeat2D Support(Xp = Xo2, Yp = Yo2, Xb = 0, Yb = 0, k = k1);
    ThreePortStepWheel2D Wheel2(X0 = Xo2, Y0 = Yo2, phi0 = 0, R1 = R21, R2 = R22, m = m2, I = i2, Color = {255, 0, 255});
    CheatedRCCB2D roll(R1 = R1, R2 = R21);
    RealSeat2D Support2(Xp = Xo3, Yp = Yo3, Xb = 0, Yb = 0, k = k2);
    ThreePortWheel2D Wheel3(X0 = Xo3, Y0 = Yo3, phi0 = 0, R = R3, m = m3, I = i3);
    ThreadRR2D Thread(k1 = 1, k2 = 1, R1 = R22, R2 = R3);
    Rod2D Palka(L = L4, m = m4, I = i4);
    Joint2D Sharnir(Xb1 = -R3, Yb1 = 0, Xb2 = -L4 / 2, Yb2 = 0);
    ForcedSlider2D Polzun(Xb = L4 / 2, Yb = 0, Xp = Xo3, Yp = Yo3, Phip = 1.57, coef = 0.1);
  equation
    connect(Wheel1.Body, Engine.Body);
    connect(Wheel1.Body, roll.Body1);
    connect(Wheel2.Body, roll.Body2);
    connect(Wheel2.Body, Support.Body);
    connect(Wheel2.Body, Thread.Body1);
    connect(Wheel3.Body, Thread.Body2);
    connect(Wheel3.Body, Support2.Body);
    connect(Wheel3.Body, Sharnir.Body1);
    connect(Palka.Body, Sharnir.Body2);
    connect(Palka.Body, Polzun.Body);
    connect(Wheel1.A, Engine.Body_out);
    connect(Wheel1.B, roll.Body1_out);
    connect(Wheel2.A, roll.Body2_out);
    connect(Wheel2.B, Support.Body_out);
    connect(Wheel2.D, Thread.Body1_out);
    connect(Wheel3.A, Thread.Body2_out);
    connect(Wheel3.B, Support2.Body_out);
    connect(Wheel3.D, Sharnir.Body1_out);
    connect(Palka.A, Sharnir.Body2_out);
    connect(Palka.B, Polzun.Body_out);
    annotation(
      experiment(StartTime = 0, StopTime = 100, Tolerance = 1e-06, Interval = 0.002));
  end SystemNumber29;

  model ForcedSlider2D
    // parametry pryamoi v prostranstve
    parameter Modelica.SIunits.Position Xp = 0;
    parameter Modelica.SIunits.Position Yp = 0;
    parameter Modelica.SIunits.Angle Phip = 0;
    parameter Modelica.SIunits.CoefficientOfFriction coef = 0;
    Modelica.SIunits.CoefficientOfFriction f;
    // parametry polzuna v tele
    parameter Modelica.SIunits.Position Xb = 0;
    parameter Modelica.SIunits.Position Yb = 0;
    // animatsiya
    parameter Modelica.SIunits.Length BoxLength = 0.25;
    parameter Modelica.Mechanics.MultiBody.Frames.Orientation orientation = Modelica.Mechanics.MultiBody.Frames.axesRotations({1, 2, 3}, {0, 0, 0}, {0, 0, 0});
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape SliderJshape(shapeType = "cylinder", length = 0.25, width = 0.1, height = 0.1, lengthDirection = {0, 0, 1}, widthDirection = {0, 1, 0}, color = {0, 0, 0}, specularCoefficient = 0.5, r = {Xp + S * cos(Phip), Yp + S * sin(Phip), 0}, R = orientation, r_shape = {0, 0, 0});
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape SliderBshape(shapeType = "box", length = BoxLength, width = 0.2, height = 0.2, lengthDirection = {cos(Phip), sin(Phip), 0}, widthDirection = {0, 0, 1}, color = {0, 0, 160}, specularCoefficient = 0.5, r = {Xp + (S - BoxLength / 2) * cos(Phip), Yp + (S - BoxLength / 2) * sin(Phip), 0}, R = orientation, r_shape = {0, 0, 0});
    // polozhenie polzuna na pryamoi
    Modelica.SIunits.Length S;
    KinematicInput Body;
    DynamicOutput Body_out;
    Modelica.SIunits.Force N;
  equation
    Xp + S * cos(Phip) = Body.X + Xb * cos(Body.Phi) - Yb * sin(Body.Phi);
    Yp + S * sin(Phip) = Body.Y + Xb * sin(Body.Phi) + Yb * cos(Body.Phi);
    if der(S) > 0.001 then
      f = coef;
    elseif der(S) < (-0.001) then
      f = -coef;
    else
      f = coef * der(S) / 0.001;
    end if;
    Body_out.Xp = Xp + S * cos(Phip);
    Body_out.Yp = Yp + S * sin(Phip);
    Body_out.Fx = (-f * N * cos(Phip)) - N * sin(Phip);
    Body_out.Fy = (-f * N * sin(Phip)) + N * cos(Phip);
    Body_out.M = 0;
  end ForcedSlider2D;

  model CheatedThreadRFB2D
    parameter Modelica.SIunits.Length R = 1;
    parameter Modelica.SIunits.Angle alpha = 0;
    parameter Real k = 1;
    parameter Modelica.SIunits.Force F = 0;
    parameter Modelica.SIunits.Length L0 = 1;
    parameter Modelica.Mechanics.MultiBody.Frames.Orientation orientation = Modelica.Mechanics.MultiBody.Frames.axesRotations({1, 2, 3}, {0, 0, 0}, {0, 0, 0});
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape Threadshape(shapeType = "cylinder", length = L_actual, width = 0.05, height = 0.05, lengthDirection = {-k * cos(alpha), -k * sin(alpha), 0}, widthDirection = {0, 0, 1}, color = {0, 0, 0}, specularCoefficient = 0.5, r = {Body.X - R * sin(alpha), Body.Y + R * cos(alpha), 0}, R = orientation, r_shape = {0, 0, 0.1});
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape Forceshape(shapeType = "cylinder", length = 1, width = 0.2, height = 0.2, lengthDirection = {-k * cos(alpha), -k * sin(alpha), 0}, widthDirection = {0, 0, 1}, color = {0.8, 0, 0}, specularCoefficient = 0.5, r = {Body.X - R * sin(alpha) - k * L_actual * cos(alpha), Body.Y + R * cos(alpha) - k * L_actual * sin(alpha), 0}, R = orientation, r_shape = {0, 0, 0.1});
    Modelica.SIunits.Length L_actual;
    KinematicInput Body;
    DynamicOutput Body_out;
  equation
    L_actual = L0 + k * Body.Phi * R;
    Body_out.Xp = Body.X - R * sin(alpha);
    Body_out.Yp = Body.Y + R * cos(alpha);
    Body_out.Fx = -k * F * cos(alpha);
    Body_out.Fy = -k * F * sin(alpha);
    Body_out.M = 0;
  end CheatedThreadRFB2D;

  model ForcedThreadTest
    parameter Modelica.SIunits.Length R1 = 1;
    parameter Modelica.SIunits.Angle phip = 0;
    parameter Modelica.SIunits.Position XO = -3;
    parameter Modelica.SIunits.Position YO = 0;
    parameter Modelica.SIunits.Position Xp0 = XO - R1 * sin(phip);
    parameter Modelica.SIunits.Position Yp0 = YO + R1 * cos(phip);
    parameter Modelica.SIunits.Mass m3 = 5;
    parameter Modelica.SIunits.MomentOfInertia I3 = m3 * R1 ^ 2 / 2;
    parameter Modelica.SIunits.Force F0 = 10;
    Wheel2D Koleso(X0 = Xp0, Y0 = Yp0, phi0 = 0, R = R1, m = m3, I = I3);
    RollCircleOnLine Roll(Xp = XO, Yp = YO, Phip = phip, R = R1);
    CheatedThreadRFB2D Nitka(R = R1, alpha = phip, k = -1, F = F0, L0 = 5);
  equation
    connect(Koleso.Body, Nitka.Body);
    connect(Koleso.Body, Roll.Body);
    connect(Koleso.A, Nitka.Body_out);
    connect(Koleso.B, Roll.Body_out);
    annotation(
      experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-06, Interval = 0.0002));
  end ForcedThreadTest;

  model CheatedThreadRS2D
    parameter Modelica.SIunits.Length R = 1;
    parameter Modelica.SIunits.Angle alpha = 0;
    parameter Real k = 1;
    parameter Modelica.SIunits.Length L0 = 1;
    parameter Modelica.Mechanics.MultiBody.Frames.Orientation orientation = Modelica.Mechanics.MultiBody.Frames.axesRotations({1, 2, 3}, {0, 0, 0}, {0, 0, 0});
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape Threadshape(shapeType = "cylinder", length = L_actual, width = 0.05, height = 0.05, lengthDirection = {-k * cos(alpha), -k * sin(alpha), 0}, widthDirection = {0, 0, 1}, color = {0, 0, 0}, specularCoefficient = 0.5, r = {Body.X - R * sin(alpha), Body.Y + R * cos(alpha), 0}, R = orientation, r_shape = {0, 0, 0.1});
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape Forceshape(shapeType = "cylinder", length = 1, width = 0.2, height = 0.2, lengthDirection = {-k * cos(alpha), -k * sin(alpha), 0}, widthDirection = {0, 0, 1}, color = {0.8, 0, 0}, specularCoefficient = 0.5, r = {Body.X - R * sin(alpha) - k * L_actual * cos(alpha), Body.Y + R * cos(alpha) - k * L_actual * sin(alpha), 0}, R = orientation, r_shape = {0, 0, 0.1});
    Modelica.SIunits.Length L_actual;
    Modelica.SIunits.Force T;
    KinematicInput Body;
    DynamicOutput Body_out;
  equation
    L_actual = L0 + k * Body.Phi * R;
    Body_out.Xp = Body.X - R * sin(alpha);
    Body_out.Yp = Body.Y + R * cos(alpha);
    Body_out.Fx = -k * T * cos(alpha);
    Body_out.Fy = -k * T * sin(alpha);
    Body_out.M = 0;
  end CheatedThreadRS2D;

  model CheatedThreadRR2D
    //1-st body on base, 2-nd body on the plane
    parameter Modelica.SIunits.Angle phi0 = 1;
    parameter Modelica.SIunits.Length R1 = 1;
    parameter Modelica.SIunits.Length R2 = 1;
    parameter Modelica.SIunits.Angle alpha = 0;
    parameter Real k1 = 1;
    parameter Real k2 = 1;
    parameter Modelica.Mechanics.MultiBody.Frames.Orientation orientation = Modelica.Mechanics.MultiBody.Frames.axesRotations({1, 2, 3}, {0, 0, 0}, {0, 0, 0});
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape Threadshape(shapeType = "cylinder", length = L_actual, width = 0.05, height = 0.05, lengthDirection = {cos(alpha), sin(alpha), 0}, widthDirection = {0, 0, 1}, color = {0, 0, 0}, specularCoefficient = 0.5, r = {Body2.X - R2 * sin(alpha), Body2.Y + R2 * cos(alpha), 0}, R = orientation, r_shape = {0, 0, 0.1});
    Modelica.SIunits.Length L_actual;
    KinematicInput Body1;
    //Based Disk
    KinematicInput Body2;
    //Rolling Disk
    Modelica.SIunits.Force T;
    DynamicOutput Body1_out;
    DynamicOutput Body2_out;
  equation
    k1 * R1 * der(Body1.Phi) = -k2 * 2 * R2 * der(Body2.Phi);
    L_actual = ((Body1.X - R1 * sin(alpha) - Body2.X + R2 * sin(alpha)) ^ 2 + (Body1.Y + R1 * cos(alpha) - Body2.Y - R2 * cos(alpha)) ^ 2) ^ 0.5;
    Body1_out.Xp = Body1.X - R1 * sin(alpha);
    Body1_out.Yp = Body1.Y + R1 * cos(alpha);
    Body1_out.Fx = -T * cos(alpha);
    Body1_out.Fy = -T * sin(alpha);
    Body1_out.M = 0;
    Body2_out.Xp = Body2.X - R2 * sin(alpha);
    Body2_out.Yp = Body2.Y + R2 * cos(alpha);
    Body2_out.Fx = T * cos(alpha);
    Body2_out.Fy = T * sin(alpha);
    Body2_out.M = 0;
    annotation(
      experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-6, Interval = 0.002));
  end CheatedThreadRR2D;

  model NewCheatedThreadTest
    parameter Modelica.SIunits.Length R1 = 1;
    parameter Modelica.SIunits.Length R2 = 5;
    parameter Modelica.SIunits.Length L0 = 10;
    parameter Modelica.SIunits.Angle phi10 = 0;
    parameter Modelica.SIunits.Angle phi20 = 0;
    parameter Modelica.SIunits.Angle phi30 = 0;
    parameter Modelica.SIunits.Angle alpha = 1.57;
    parameter Modelica.SIunits.Position XO = 0;
    parameter Modelica.SIunits.Position YO = 0;
    parameter Modelica.SIunits.Position Xp0 = XO - k1 * R1 * sin(alpha) + R2 * sin(alpha) - L0 * cos(alpha);
    parameter Modelica.SIunits.Position Yp0 = YO + k1 * R1 * cos(alpha) - R2 * cos(alpha) - L0 * sin(alpha);
    parameter Modelica.SIunits.Position Xpl0 = XO - k1 * R1 * sin(alpha) + 2 * R2 * sin(alpha) - L0 * cos(alpha);
    parameter Modelica.SIunits.Position Ypl0 = YO + k1 * R1 * cos(alpha) - 2 * R2 * cos(alpha) - L0 * sin(alpha);
    parameter Modelica.SIunits.Mass m1 = 1;
    parameter Modelica.SIunits.Mass m2 = 3;
    parameter Modelica.SIunits.Mass m3 = 5;
    parameter Modelica.SIunits.MomentOfInertia I1 = m1 * R1 ^ 2 / 2;
    parameter Modelica.SIunits.MomentOfInertia I2 = m2 * R2 ^ 2 / 2;
    parameter Real k1 = 1;
    parameter Real k2 = 1;
    Seat2D Opora(Xp = XO, Yp = YO, Xb = 0, Yb = 0);
    Wheel2D Koleso(X0 = XO, Y0 = YO, R = R1, m = m1, I = I1);
    CheatedThreadRR2D Nitka(R1 = R1, R2 = R2, alpha = alpha, k1 = k1, k2 = k2);
    Wheel2D Koleso2(X0 = Xp0, Y0 = Yp0, R = R2, m = m2, I = I2);
    RollCircleOnLine Roll(Xp = Xpl0, Yp = Ypl0, Phip = alpha, R = R2);
  equation
    connect(Koleso.Body, Opora.Body);
    connect(Koleso.Body, Nitka.Body1);
    connect(Koleso2.Body, Nitka.Body2);
    connect(Koleso2.Body, Roll.Body);
    connect(Koleso.A, Opora.Body_out);
    connect(Koleso.B, Nitka.Body1_out);
    connect(Koleso2.A, Roll.Body_out);
    connect(Koleso2.B, Nitka.Body2_out);
    annotation(
      experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-06, Interval = 0.002));
  end NewCheatedThreadTest;

  model StepBlockTest
    parameter Modelica.SIunits.Length R1 = 1;
    parameter Modelica.SIunits.Length R2 = 5;
    parameter Modelica.SIunits.Length R3 = 3;
    parameter Modelica.SIunits.Length L0 = 10;
    parameter Modelica.SIunits.Angle phi10 = 0;
    parameter Modelica.SIunits.Angle phi20 = 0;
    parameter Modelica.SIunits.Angle phi30 = 0;
    parameter Modelica.SIunits.Angle alpha = 1.57;
    parameter Modelica.SIunits.Position XO = 0;
    parameter Modelica.SIunits.Position YO = 0;
    parameter Modelica.SIunits.Position Xp0 = XO - k1 * R1 * sin(alpha) + R2 * sin(alpha) - L0 * cos(alpha);
    parameter Modelica.SIunits.Position Yp0 = YO + k1 * R1 * cos(alpha) - R2 * cos(alpha) - L0 * sin(alpha);
    parameter Modelica.SIunits.Position Xpl0 = XO - k1 * R1 * sin(alpha) + 2 * R2 * sin(alpha) - L0 * cos(alpha);
    parameter Modelica.SIunits.Position Ypl0 = YO + k1 * R1 * cos(alpha) - 2 * R2 * cos(alpha) - L0 * sin(alpha);
    parameter Modelica.SIunits.Mass m1 = 1;
    parameter Modelica.SIunits.Mass m2 = 3;
    parameter Modelica.SIunits.Mass m3 = 5;
    parameter Modelica.SIunits.MomentOfInertia I1 = m1 * R1 ^ 2 / 2;
    parameter Modelica.SIunits.MomentOfInertia I2 = m2 * R2 ^ 2 / 2;
    parameter Real k1 = -1;
    parameter Real k2 = -1;
    parameter Modelica.SIunits.Force F0 = 15;
    Seat2D Opora(Xp = XO, Yp = YO, Xb = 0, Yb = 0);
    Wheel2D Koleso(X0 = XO, Y0 = YO, R = R1, m = m1, I = I1);
    CheatedThreadRR2D Nitka(R1 = R1, R2 = R2, alpha = alpha, k1 = k1, k2 = k2);
    ThreePortStepWheel2D Koleso2(X0 = Xp0, Y0 = Yp0, R1 = R2, R2 = R3, m = m2, I = I2);
    RollCircleOnLine Roll(Xp = Xpl0, Yp = Ypl0, Phip = alpha, R = R2);
    CheatedThreadRFB2D Nitka2(R = R3, alpha = alpha, k = 1, F = F0, L0 = 5);
  equation
    connect(Koleso.Body, Opora.Body);
    connect(Koleso.Body, Nitka.Body1);
    connect(Koleso2.Body, Nitka.Body2);
    connect(Koleso2.Body, Roll.Body);
    connect(Koleso2.Body, Nitka2.Body);
    connect(Koleso.A, Opora.Body_out);
    connect(Koleso.B, Nitka.Body1_out);
    connect(Koleso2.A, Roll.Body_out);
    connect(Koleso2.B, Nitka.Body2_out);
    connect(Koleso2.D, Nitka2.Body_out);
    annotation(
      experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002));
  end StepBlockTest;

  model lab4_myown
    /*parameter Modelica.SIunits.Length R11 = 2;
    parameter Modelica.SIunits.Length R12 = 1.5;
    parameter Modelica.SIunits.Angle phip = 0;
    parameter Modelica.SIunits.Position XO = -3;
    parameter Modelica.SIunits.Position YO = 0;
    parameter Modelica.SIunits.Position Xp0 = XO - R11 * sin(phip);
    parameter Modelica.SIunits.Position Yp0 = YO + R11 * cos(phip);
    parameter Modelica.SIunits.Mass m3 = 5;
    parameter Modelica.SIunits.MomentOfInertia I3 = m3 * R11 ^ 2 / 2;
    parameter Modelica.SIunits.Force F0 = 10;
    ThreePortStepWheel2D Koleso(X0 = Xp0, Y0 = Yp0, phi0 = 0, R1 = R11, R2 = R12, m = m3, I = I3);
    RollCircleOnLine Roll(Xp = XO, Yp = YO, Phip = phip, R = R11);
    CheatedThreadRFB2D Nitka(R = R12, alpha = phip, k = -1, F = F0, L0 = 5);
  equation
    connect(Koleso.Body, Nitka.Body);
    connect(Koleso.Body, Roll.Body);
    connect(Koleso.A, Nitka.Body_out);
    connect(Koleso.B, Roll.Body_out);*/
    parameter Modelica.SIunits.Length L1 = 3;
    parameter Modelica.SIunits.Length R1 = 0.8;
    parameter Modelica.SIunits.Length R2 = 0.4;
    parameter Modelica.SIunits.Length R31 = 0.8;
    parameter Modelica.SIunits.Length R32 = 0.4;
    parameter Modelica.SIunits.Length L_nitka1 = 2.5;
    parameter Modelica.SIunits.Length L_nitka2 = 3.5;
    parameter Modelica.SIunits.Angle phi10 = 0;
    parameter Modelica.SIunits.Position Xp0 = R1 * cos(0) + L1 * cos(phi10);
    parameter Modelica.SIunits.Position Yp0 = R1 * sin(0) + L1 * sin(phi10);
    parameter Modelica.SIunits.Position Y1 = 0 + L_nitka1;
    parameter Modelica.SIunits.Position X2 = 0 + L_nitka2;
    parameter Modelica.SIunits.Position Y2 = Y1 + R2 / 2;
    parameter Modelica.SIunits.Angle alpha1 = 1.57;
    parameter Modelica.SIunits.Mass m1 = 1;
    parameter Modelica.SIunits.Mass m2 = 3;
    parameter Modelica.SIunits.Mass m3 = 1.5;
    parameter Modelica.SIunits.Mass m4 = 3.5;
    parameter Modelica.SIunits.MomentOfInertia I1 = m1 * L1 ^ 2 / 12;
    parameter Modelica.SIunits.MomentOfInertia I2 = m2 * R1 ^ 2 / 2;
    parameter Modelica.SIunits.MomentOfInertia I3 = m3 * R2 ^ 2 / 2;
    parameter Modelica.SIunits.MomentOfInertia I4 = m4 * R31 ^ 2 / 2;
    parameter Modelica.SIunits.Force F0 = 8;
    parameter Real k1 = -1;
    parameter Real k2 = -1;
    
    RealSeat2D Opora1(Xp = 0, Yp = 0, Xb = 0, Yb = 0);
    RealSeat2D Opora2(Xp = -R1 + R2, Yp = Y1, Xb = 0, Yb = 0);
    //Thread usu
    ThreadRR2D Nitka1(R1 = R1, R2 = R2, k1 = k1, k2 = -k2);
    //RRBP
    CheatedThreadRRBPC2D Nitka2(R1 = R32, R2 = R2, alpha = 0, k1 = k1);
    
    CheatedThreadRFB2D Nitka3(R = R32, alpha = 0, k = -1, F = 5, L0 = 2);
    ThreePortWheel2D Koleso1(X0 = 0, Y0 = 0, R = R1, m = m2, I = I2);
    ThreePortWheel2D Koleso2(X0 = 0, Y0 = Y1, R = R2, m = m3, I = I3);
    ThreePortStepWheel2D Koleso3(X0 = X2, Y0 = Y2, phi0 = 0, R1 = R31, R2 = R32, m = m4, I = I4);
    RollCircleOnLine Roll(Xp = X2, Yp = Y2 - R31, Phip = 0, R = R31);
    Rod2D Palka1(phi0 = phi10, L = L1, m = m1, I = I1);
    Joint2D Sharnir1(Xb1 = 0, Yb1 = R1, Xb2 = -L1 / 2, Yb2 = 0);
    PistonEngine Engine(Xb = L1 / 2, Yb = 0, Xp = Xp0, Yp = Yp0, Phip = 0, F0 = F0);
  equation
    connect(Koleso1.Body, Opora1.Body);
    connect(Koleso1.Body, Nitka1.Body1);
    connect(Koleso1.Body, Sharnir1.Body1);
    connect(Palka1.Body, Sharnir1.Body2);
    connect(Palka1.Body, Engine.Body);
    connect(Koleso2.Body, Nitka1.Body2);
    connect(Koleso2.Body, Nitka2.Body1);
    connect(Koleso2.Body, Opora2.Body);
    connect(Koleso3.Body, Nitka2.Body2);
    connect(Koleso3.Body, Nitka3.Body);
    connect(Koleso3.Body, Roll.Body);
    
    connect(Koleso1.A, Opora1.Body_out);
    connect(Koleso1.B, Nitka1.Body1_out);
    connect(Koleso1.D, Sharnir1.Body1_out);
    connect(Palka1.A, Sharnir1.Body2_out);
    connect(Palka1.B, Engine.Body_out);
    connect(Koleso2.A, Nitka1.Body2_out);
    connect(Koleso2.B, Opora2.Body_out);
    connect(Koleso2.D, Nitka2.Body1_out);
    connect(Koleso3.A, Nitka2.Body2_out);
    connect(Koleso3.B, Nitka3.Body_out);
    connect(Koleso3.D, Roll.Body_out);
  end lab4_myown;
end DynamicSandBox;
