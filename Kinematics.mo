package Kinematics
  import Modelica.SIunits;

  partial model Body2D
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

  partial model Support2D
    // Koordinaty opori v prostranstve
    parameter Modelica.SIunits.Position Xo = 0;
    parameter Modelica.SIunits.Position Yo = 0;
    // Koordinaty opori v tele
    parameter Modelica.SIunits.Position Xb = 0;
    parameter Modelica.SIunits.Position Yb = 0;
    parameter Modelica.Mechanics.MultiBody.Frames.Orientation orientation = Modelica.Mechanics.MultiBody.Frames.axesRotations({1, 2, 3}, {0, 0, 0}, {0, 0, 0});
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape Supportshape(shapeType = "cylinder", length = 0.25, width = 0.1, height = 0.1, lengthDirection = {0, 0, 1}, widthDirection = {0, 1, 0}, color = {0, 0, 0}, specularCoefficient = 0.5, r = {Xo, Yo, 0}, R = orientation, r_shape = {0, 0, 0});
    KinematicInput Body;
  equation
    Body.X + Xb * cos(Body.Phi) - Yb * sin(Body.Phi) = Xo;
    Body.Y + Xb * sin(Body.Phi) + Yb * cos(Body.Phi) = Yo;
  end Support2D;

  model PalkaNaOpore
    parameter Modelica.SIunits.Length L1 = 4;
    parameter Modelica.SIunits.AngularVelocity Omega = 1;
    parameter Modelica.SIunits.Angle phi10 = 1;
    Body2D Palka(phi0 = 0, L = L1);
    Support2D Opora(Xo = 1, Yo = 2, Xb = -L1 / 2, Yb = 0);
  equation
    connect(Palka.Body, Opora.Body);
    Palka.Phi = phi10 * sin(Omega * time);
    annotation(
      experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-6, Interval = 0.0002));
  end PalkaNaOpore;

  partial model Joint2D
    // Koordinaty opori v prostranstve
    parameter Modelica.SIunits.Position Xb1 = 0;
    parameter Modelica.SIunits.Position Yb1 = 0;
    // Koordinaty opori v tele
    parameter Modelica.SIunits.Position Xb2 = 0;
    parameter Modelica.SIunits.Position Yb2 = 0;
    parameter Modelica.Mechanics.MultiBody.Frames.Orientation orientation = Modelica.Mechanics.MultiBody.Frames.axesRotations({1, 2, 3}, {0, 0, 0}, {0, 0, 0});
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape Jointshape(shapeType = "cylinder", length = 0.3, width = 0.1, height = 0.1, lengthDirection = {0, 0, 1}, widthDirection = {0, 1, 0}, color = {0, 0, 0}, specularCoefficient = 0.5, r = {XSh, YSh, 0}, R = orientation, r_shape = {0, 0, 0});
    Modelica.SIunits.Position XSh;
    Modelica.SIunits.Position YSh;
    KinematicInput Body1;
    KinematicInput Body2;
  equation
    XSh = Body1.X + Xb1 * cos(Body1.Phi) - Yb1 * sin(Body1.Phi);
    YSh = Body1.Y + Xb1 * sin(Body1.Phi) + Yb1 * cos(Body1.Phi);
    XSh = Body2.X + Xb2 * cos(Body2.Phi) - Yb2 * sin(Body2.Phi);
    YSh = Body2.Y + Xb2 * sin(Body2.Phi) + Yb2 * cos(Body2.Phi);
  end Joint2D;

  model DvePalkiNaOpore
    parameter Modelica.SIunits.Length L1 = 4;
    parameter Modelica.SIunits.Length L2 = 2;
    parameter Modelica.SIunits.AngularVelocity Omega1 = 1;
    parameter Modelica.SIunits.AngularVelocity Omega2 = 10;
    parameter Modelica.SIunits.Angle phi10 = 1;
    parameter Modelica.SIunits.Angle phi20 = 1;
    Body2D Palka1(phi0 = phi10, L = L1);
    Body2D Palka2(phi0 = phi20, L = L2);
    Support2D Opora(Xo = 1, Yo = 2, Xb = -L1 / 2, Yb = 0);
    Joint2D Ssharnir(Xb1 = L1 / 2, Yb1 = 0, Xb2 = -L2 / 2, Yb2 = 0);
  equation
    connect(Palka1.Body, Opora.Body);
    connect(Palka1.Body, Ssharnir.Body1);
    connect(Palka2.Body, Ssharnir.Body2);
    Palka1.Phi = phi10 * sin(Omega1 * time);
    Palka2.Phi = phi20 + Omega2 * time;
    annotation(
      experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-06, Interval = 0.0002));
  end DvePalkiNaOpore;

  partial model Slider2D
    // Koordinaty opori v prostranstve
    parameter Modelica.SIunits.Position Xp = 0;
    parameter Modelica.SIunits.Position Yp = 0;
    parameter Modelica.SIunits.Angle Phip = 0;
    // Koordinaty opori v tele
    parameter Modelica.SIunits.Position Xb = 0;
    parameter Modelica.SIunits.Position Yb = 0;
    parameter Modelica.SIunits.Length BoxLength = 0.25;
    parameter Modelica.Mechanics.MultiBody.Frames.Orientation orientation = Modelica.Mechanics.MultiBody.Frames.axesRotations({1, 2, 3}, {0, 0, 0}, {0, 0, 0});
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape SliderJshape(shapeType = "cylinder", length = BoxLength, width = 0.1, height = 0.1, lengthDirection = {0, 0, 1}, widthDirection = {0, 1, 0}, color = {0, 0, 0}, specularCoefficient = 0.5, r = {Xp + a * cos(Phip), Yp + a * sin(Phip), 0}, R = orientation, r_shape = {0, 0, 0});
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape SliderBshape(shapeType = "box", length = 0.25, width = 0.2, height = 0.2, lengthDirection = {cos(Phip), sin(Phip), 0}, widthDirection = {0, 0, 1}, color = {0, 0, 130}, specularCoefficient = 0.5, r = {Xp + (a - BoxLength / 2) * cos(Phip), Yp + (a - BoxLength / 2) * sin(Phip), 0}, R = orientation, r_shape = {0, 0, 0});
    Modelica.SIunits.Length a;
    KinematicInput Body;
  equation
    Body.X + Xb * cos(Body.Phi) - Yb * sin(Body.Phi) = Xp + a * cos(Phip);
    Body.Y + Xb * sin(Body.Phi) + Yb * cos(Body.Phi) = Yp + a * sin(Phip);
  end Slider2D;

  model SliderTest
    parameter Modelica.SIunits.Length L1 = 1;
    parameter Modelica.SIunits.Length L2 = 5;
    parameter Modelica.SIunits.AngularVelocity Omega1 = 1;
    parameter Modelica.SIunits.Angle phi10 = 1;
    parameter Modelica.SIunits.Angle phi20 = 1;
    parameter Modelica.SIunits.Position Xp0 = L1 * cos(phi10) + L2 * cos(phi20);
    parameter Modelica.SIunits.Position Yp0 = L1 * sin(phi10) + L2 * sin(phi20);
    parameter Modelica.SIunits.Angle PhiP = 1;
    Rod2D Palka1(phi0 = phi10, L = L1);
    Rod2D Palka2(phi0 = phi20, L = L2, Color = {0, 255, 0});
    Support2D Opora(Xo = 1, Yo = 2, Xb = -L1 / 2, Yb = 0);
    Joint2D Ssharnir(Xb1 = L1 / 2, Yb1 = 0, Xb2 = -L2 / 2, Yb2 = 0);
    Clutch Polzun(Xb = L2 / 2, Yb = 0, Xp = Xp0, Yp = Yp0, Phip = PhiP);
  equation
    connect(Palka1.Body, Opora.Body);
    connect(Palka1.Body, Ssharnir.Body1);
    connect(Palka2.Body, Ssharnir.Body2);
    connect(Palka2.Body, Polzun.Body);
    Palka1.Phi = phi10 + Omega1 * time;
    annotation(
      experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-06, Interval = 0.0002));
  end SliderTest;

  partial model RollCircleOnLine
    parameter Modelica.SIunits.Position Xp = 0;
    parameter Modelica.SIunits.Position Yp = 0;
    parameter Modelica.SIunits.Angle Phip = 0;
    parameter Modelica.SIunits.Length R = 1;
    Modelica.SIunits.Length S;
    KinematicInput Body;
    parameter Modelica.Mechanics.MultiBody.Frames.Orientation orientation = Modelica.Mechanics.MultiBody.Frames.axesRotations({1, 2, 3}, {0, 0, 0}, {0, 0, 0});              
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape BoxShape(shapeType = "box", length = 6*R, width = 0.3, height = 0.05, lengthDirection = {cos(Phip), sin(Phip), 0},     widthDirection = {0, 0, 1}, color = {150, 0, 0}, specularCoefficient = 0.5, r = {Xp + (0.025) * sin(Phip) - 3*R * cos(Phip), Yp - (0.025) * cos(Phip) - 3*R * sin(Phip), 0}, R = orientation, r_shape = {0, 0, 0.15});
  equation
    Body.X = Xp - R * sin(Phip) + S * cos(Phip);
    Body.Y = Yp + R * cos(Phip) + S * sin(Phip);
    der(S) = -R * der(Body.Phi);
  end RollCircleOnLine;

  partial model Rod2D
    extends Kinematics.Body2D;
    parameter Modelica.SIunits.Length L = 1;
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape Bodyshape(shapeType = "box", length = L, width = 0.05, height = 0.05, lengthDirection = {cos(Phi), sin(Phi), 0}, widthDirection = {0, 0, 1}, color = Color, specularCoefficient = 0.5, r = {X - L / 2 * cos(Phi), Y - L / 2 * sin(Phi), 0}, R = orientation, r_shape = {0, 0, 0.15});
  end Rod2D;

  partial model Wheel2D
    extends Kinematics.Body2D;
    parameter Modelica.SIunits.Length R = 1;
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape Bodyshape1(shapeType = "cylinder", length = 0.05, width = 2 * R, height = 2 * R, lengthDirection = {0, 0, 1}, widthDirection = {cos(Phi), sin(Phi), 0}, color = Color, specularCoefficient = 0.5, r = {X, Y, 0}, R = orientation, r_shape = {0, 0, 0.15});
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape Bodyshape2(shapeType = "box", length = 0.1, width = R, height = R, lengthDirection = {0, 0, 1}, widthDirection = {cos(Phi), sin(Phi), 0}, color = Color * 0.7, specularCoefficient = 0.5, r = {X, Y, 0}, R = orientation, r_shape = {0, 0, 0.15});
  end Wheel2D;

  model RollTest
    parameter Modelica.SIunits.Length L1 = 1;
    parameter Modelica.SIunits.Length L2 = 5;
    parameter Modelica.SIunits.Length R1 = 2;
    parameter Modelica.SIunits.AngularVelocity Omega1 = 2;
    parameter Modelica.SIunits.Angle phi10 = 0;
    parameter Modelica.SIunits.Angle phi20 = 0;
    parameter Modelica.SIunits.Angle phi30 = 0;
    parameter Modelica.SIunits.Angle phip = 0.7;
    parameter Modelica.SIunits.Position XO = -3;
    parameter Modelica.SIunits.Position YO = 0;
    parameter Modelica.SIunits.Position Xp0 = XO + L1 * cos(phi10) + L2 * cos(phi20) + R1 * sin(phip);
    parameter Modelica.SIunits.Position Yp0 = YO + L1 * sin(phi10) + L2 * sin(phi20) - R1 * cos(phip);
    
    Rod2D Palka1(phi0 = phi10, L = L1);
    Rod2D Palka2(phi0 = phi20, L = L2, Color = {0, 255, 0});
    Wheel2D Koleso(phi0 = phi30, R = R1, Color = {0, 255, 255});
    Support2D Opora(Xo = XO, Yo = YO, Xb = -L1 / 2, Yb = 0);
    Joint2D Ssharnir(Xb1 = L1 / 2, Yb1 = 0, Xb2 = -L2 / 2, Yb2 = 0);
    //Slider2D Polzun(Xb = L2 / 2, Yb = 0, Xp = Xp0, Yp = Yp0, Phip = PhiP);
    Joint2D Ssharnir2(Xb1 = L2 / 2, Yb1 = 0, Xb2 = 0, Yb2 = 0);
    RollCircleOnLine Roll(Xp = Xp0, Yp = Yp0, Phip = phip, R = R1);
  equation
    connect(Palka1.Body, Opora.Body);
    connect(Palka1.Body, Ssharnir.Body1);
    connect(Palka2.Body, Ssharnir.Body2);
    connect(Palka2.Body, Ssharnir2.Body1);
    connect(Koleso.Body, Ssharnir2.Body2);
    connect(Koleso.Body, Roll.Body);
    Palka1.Phi = phi10 + Omega1 * time;
    annotation(
      experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-06, Interval = 0.002));
  end RollTest;

  model RollTest0
    parameter Modelica.SIunits.Position Xp = 0;
    parameter Modelica.SIunits.Position Yp = 0;
    parameter Modelica.SIunits.Angle Phip = 1;
    parameter Modelica.SIunits.Length R = 1;
    Modelica.SIunits.Length S(start = 0);
    parameter Modelica.SIunits.AngularVelocity Omega = 1;
    parameter Modelica.SIunits.Angle phi0 = 0;
    parameter Real Color[3] = {255, 0, 0};
    parameter Modelica.Mechanics.MultiBody.Frames.Orientation orientation = Modelica.Mechanics.MultiBody.Frames.axesRotations({1, 2, 3}, {0, 0, 0}, {0, 0, 0});
    Modelica.SIunits.Position X;
    Modelica.SIunits.Position Y;
    Modelica.SIunits.Angle Phi(start = phi0);
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape Bodyshape1(shapeType = "cylinder", length = 0.05, width = 2 * R, height = 2 * R, lengthDirection = {0, 0, 1}, widthDirection = {cos(Phi), sin(Phi), 0}, color = Color, specularCoefficient = 0.5, r = {X, Y, 0}, R = orientation, r_shape = {0, 0, 0.15});
    Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape Bodyshape2(shapeType = "box", length = 0.1, width = R, height = R, lengthDirection = {0, 0, 1}, widthDirection = {cos(Phi), sin(Phi), 0}, color = Color * 0.7, specularCoefficient = 0.5, r = {X, Y, 0}, R = orientation, r_shape = {0, 0, 0.15});
  equation
    Phi = Omega * time + phi0;
    X = Xp - R * sin(Phip) + S * cos(Phip);
    Y = Yp + R * cos(Phip) + S * sin(Phip);
    der(S) = -R * der(Phi);
    annotation(
      experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-06, Interval = 0.0002));
  end RollTest0;

  model RollTest1
    parameter Modelica.SIunits.Length L1 = 4;
      parameter Modelica.SIunits.Length L2 = 2;
    parameter Modelica.SIunits.Length R1 = 1;
    parameter Modelica.SIunits.AngularVelocity Omega1 = 3;
    parameter Modelica.SIunits.AngularVelocity Omega2 = 1;
     parameter Modelica.SIunits.AngularVelocity Omega3 = 5;
    parameter Modelica.SIunits.Angle phi10 = 0;
    parameter Modelica.SIunits.Angle phi20 = 0;
      parameter Modelica.SIunits.Angle phi30 = 0;
    parameter Modelica.SIunits.Angle phip = 1;
    parameter Modelica.SIunits.Position Xp0 = 1;
    parameter Modelica.SIunits.Position Yp0 = 2;
    parameter Modelica.SIunits.Angle PhiP = 1;
    Rod2D Palka1(phi0 = phi10, L = L1);
    Rod2D Palka2(phi0 = phi30, L = L2);
    Wheel2D Koleso(phi0 = phi20, R = R1, Color = {0, 255, 255});
    Joint2D Ssharnir(Xb1 = L1 / 2, Yb1 = 0, Xb2 = 0, Yb2 = 0);
    Joint2D Ssharnir2(Xb1 = -L1 / 2, Yb1 = 0, Xb2 = L2/2, Yb2 = 0);
    RollCircleOnLine Roll(Xp = Xp0, Yp = Yp0, Phip = phip, R = R1);
  equation
    connect(Palka1.Body, Ssharnir.Body1);
    connect(Koleso.Body, Ssharnir.Body2);
    connect(Palka1.Body, Ssharnir2.Body1);
    connect(Palka2.Body, Ssharnir2.Body2);
    connect(Koleso.Body, Roll.Body);
    Palka1.Phi = phi10 - Omega1 * time;
    Koleso.Phi = phi20 + Omega2 * time;
    Palka2.Phi = phi30 - sin(Omega3 * time);
    annotation(
      experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-06, Interval = 0.0002));
  end RollTest1;

  model lab2_myown
    parameter Modelica.SIunits.Length L1 = 0.5;
    parameter Modelica.SIunits.Length L2 = 2;
    parameter Modelica.SIunits.Length L3 = 2;
    parameter Modelica.SIunits.Length L4 = 0.7;
    parameter Modelica.SIunits.Length R1 = 0.7;
    parameter Modelica.SIunits.AngularVelocity Omega1 = 2;
    parameter Modelica.SIunits.Angle phi10 = 0.5;
    parameter Modelica.SIunits.Angle phi20 = 2;
    parameter Modelica.SIunits.Angle phi30 = 0.3;
    parameter Modelica.SIunits.Angle phi40 = 4.5;
    parameter Modelica.SIunits.Angle phip0 = 0;
    parameter Modelica.SIunits.Angle phip01 = 1.57;
    parameter Modelica.SIunits.Position XO = -3;
    parameter Modelica.SIunits.Position YO = 0;
    //parameter Modelica.SIunits.Position Xp0 = XO + L1 * cos(phi10) + L2 * cos(phi20) + L3 * cos(phi30) + L4 * cos(phi40) + R1 * sin(phip0);
    parameter Modelica.SIunits.Position Xp0 = XO + L3 * cos(phi30) + L4 * cos(phi40) + R1 * sin(phip0);
    parameter Modelica.SIunits.Position Yp0 = YO + L1 * sin(phi10) + L2 * sin(phi20) + L3 * sin(phi30) + L4 * sin(phi40) - R1 * cos(phip0);
    parameter Modelica.SIunits.Position Xp01 = XO + L1 * cos(phi10) + L2 * cos(phi20);
    parameter Modelica.SIunits.Position Yp01 = YO + L1 * sin(phi10) + L2 * sin(phi20);
    
    Rod2D Palka1(phi0 = phi10, L = L1, Color = {255, 0, 0});
    Rod2D Palka2(phi0 = phi20, L = L2, Color = {0, 255, 0});
    Rod2D Palka3(phi0 = phi30, L = L3, Color = {0, 0, 255});
    Wheel2D Koleso(phi0 = phi40, R = R1, Color = {0, 255, 255});
    Support2D Opora(Xo = XO, Yo = YO, Xb = -L1 / 2, Yb = 0);
    Joint2D Ssharnir(Xb1 = L1 / 2, Yb1 = 0, Xb2 = -L2 / 2, Yb2 = 0);
    Joint2D SsharnirC(Xb1 = L2 / 2, Yb1 = 0, Xb2 = -L3 / 2, Yb2 = 0);
    Slider2D Polzun(Xb = L2 / 2, Yb = 0, Xp = Xp01, Yp = Yp01, Phip = phip01);
    Joint2D Ssharnir2(Xb1 = L3 / 2, Yb1 = 0, Xb2 = -R1, Yb2 = 0);
    RollCircleOnLine Roll(Xp = Xp0, Yp = Yp0, Phip = phip0, R = R1);
  equation
    connect(Palka1.Body, Opora.Body);
    connect(Palka1.Body, Ssharnir.Body1);
    connect(Palka2.Body, Ssharnir.Body2);
    connect(Palka2.Body, Polzun.Body);
    connect(Palka2.Body, SsharnirC.Body1);
    connect(Palka3.Body, SsharnirC.Body2);
    connect(Palka3.Body, Ssharnir2.Body1);
    connect(Koleso.Body, Ssharnir2.Body2);
    connect(Koleso.Body, Roll.Body);
    Palka1.Phi = phi10 + Omega1 * time;
  
  end lab2_myown;
end Kinematics;
