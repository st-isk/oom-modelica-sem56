package labs_OOM

  package lab_2
  model func1
      Real x(start = 10);
      Real v(start = 0);
      Real F;
      Real flag(start = 0);
      parameter Real K = 9;
      parameter Real k2 = 1;
      parameter Real k3 = 5;
      parameter Real k4 = 13;
      parameter Real a = 5;
    equation
      if flag == 0 then
        if x < a then
          F = -K;
          flag = 0;
        else
          F = K;
          flag = 1;
        end if;
      else
        if x > (-a) then
          F = K;
          flag = 1;
        else
          F = -K;
          flag = 0;
        end if;
      end if;
      der(x) = v;
      der(v) = (-F) - k2 * x + k3 * sin(k4 * time);
    end func1;

    model func2
    Real x(start=10);
    Real v(start=0);
    Real F;
    Real flag(start=0);
    parameter Real K = 5;
    parameter Real k2=2;
    parameter Real k3=20;
    parameter Real k4=1;
    parameter Real a=7;
    parameter Real b=0.5;
    equation
    if (flag < 0.5) then
    if (x > -b * a) then
    flag = 1;
    F = 0;
    else
    F = -K;
    flag = 0;
    end if;
    else
    if (flag > 0.5 and flag < 1.5) then
    if (x > -a and x < a) then
    F = 0;
    flag = 1;
    else
    if (x > a) then
    F = K;
    flag = 2;
    else
    F = -K;
    flag = 0;
    end if;
    end if;
    else
    if (x > b * a) then
    F = K;
    flag = 2;
    else
    F = 0;
    flag = 1;
    end if;
    end if;
    end if;
    der(v)= -F -k2 * x + k3 * sin(k4*time);
    der(x) = v;
    end func2;

    model func3
    Real x(start = -5);
    Real v(start = 1);
    Real F;
    parameter Real K = 4;
    parameter Real k2 = 3;
    parameter Real k3 = 4;
    parameter Real k4 = 5;
    parameter Real a = 1;
    parameter Real b = 3;
    parameter Real d = 3;
    equation
    if (x < -d) then
    F = -K;
    elseif (x < -a) then
    F = K * (x + d) / d - K;
    elseif (x < a) then
    F = 0;
    elseif (x < d) then
    F = K * (x + d) / d - K;
    else
    F = K;
    end if;
    der(x) = v;
    der(v) = (-F) - k2 * x + k3 * sin(k4 * time);
    end func3;
  end lab_2;

  package lab_3
    model scheme_1
      Modelica.Electrical.Analog.Basic.Resistor resistor1(R = 2000) annotation(
        Placement(visible = true, transformation(origin = {10, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Electrical.Analog.Basic.Resistor resistor2(R = 3000) annotation(
        Placement(visible = true, transformation(origin = {10, -16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Electrical.Analog.Basic.Inductor inductor(L = 50) annotation(
        Placement(visible = true, transformation(origin = {10, 48}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Electrical.Analog.Basic.Capacitor capacitor(C = 0.000000001) annotation(
        Placement(visible = true, transformation(origin = {80, 12}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
      Modelica.Electrical.Analog.Sources.SineVoltage sineVoltage(V = 220, freqHz = 50) annotation(
        Placement(visible = true, transformation(origin = {-74, 2}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Modelica.Electrical.Analog.Basic.Ground ground annotation(
        Placement(visible = true, transformation(origin = {-74, -56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      labs_OOM.lab_3.ourResistor ourResistor(R = 1000) annotation(
        Placement(visible = true, transformation(origin = {-32, 36}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(resistor1.n, capacitor.p) annotation(
        Line(points = {{20, 22}, {80, 22}}, color = {0, 0, 255}));
      connect(inductor.n, capacitor.p) annotation(
        Line(points = {{20, 48}, {80, 48}, {80, 22}}, color = {0, 0, 255}));
      connect(capacitor.n, resistor2.n) annotation(
        Line(points = {{80, 2}, {20, 2}, {20, -16}, {20, -16}}, color = {0, 0, 255}));
      connect(resistor2.p, sineVoltage.p) annotation(
        Line(points = {{0, -16}, {-74, -16}, {-74, -8}}, color = {0, 0, 255}));
      connect(resistor2.p, ground.p) annotation(
        Line(points = {{0, -16}, {-74, -16}, {-74, -46}, {-74, -46}}, color = {0, 0, 255}));
      connect(sineVoltage.n, ourResistor.p) annotation(
        Line(points = {{-74, 12}, {-42, 12}, {-42, 36}, {-42, 36}}, color = {0, 0, 255}));
      connect(ourResistor.n, inductor.p) annotation(
        Line(points = {{-22, 36}, {0, 36}, {0, 48}, {0, 48}}, color = {0, 0, 255}));
      connect(ourResistor.n, resistor1.p) annotation(
        Line(points = {{-22, 36}, {0, 36}, {0, 22}, {0, 22}}, color = {0, 0, 255}));
    end scheme_1;

    model ourResistor "Ideal linear electrical resistor"
      parameter Modelica.SIunits.Resistance R(start = 1) "Resistance at temperature T_ref";
      parameter Modelica.SIunits.Temperature T_ref = 300.15 "Reference temperature";
      //parameter SI.LinearTemperatureCoefficient alpha=0
      parameter Modelica.SIunits.Current I0 = 1;
      parameter Real alpha = 10 "Temperature coefficient of resistance (R_actual = R*(1 + alpha*(T_heatPort - T_ref))";
      extends Modelica.Electrical.Analog.Interfaces.OnePort;
      extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(T = T_ref);
      Modelica.SIunits.Resistance R_actual "Actual resistance = R*(1 + alpha*(T_heatPort - T_ref))";
    equation
      assert(1 + alpha * (T_heatPort - T_ref) >= Modelica.Constants.eps, "Temperature outside scope of model!");
      R_actual = R * (1 + alpha * (i / I0));
      v = R_actual * i;
      LossPower = v * i;
      annotation(
        Documentation(info = "<html>
    <p>The linear resistor connects the branch voltage <em>v</em> with the branch current <em>i</em> by <em>i*R = v</em>. The Resistance <em>R</em> is allowed to be positive, zero, or negative.</p>
    </html>", revisions = "<html>
    <ul>
    <li><em> August 07, 2009   </em>
         by Anton Haumer<br> temperature dependency of resistance added<br>
         </li>
    <li><em> March 11, 2009   </em>
         by Christoph Clauss<br> conditional heat port added<br>
         </li>
    <li><em> 1998   </em>
         by Christoph Clauss<br> initially implemented<br>
         </li>
    </ul>
    </html>"),
        Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(extent = {{-70, 30}, {70, -30}}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Line(points = {{-90, 0}, {-70, 0}}, color = {0, 0, 255}), Line(points = {{70, 0}, {90, 0}}, color = {0, 0, 255}), Text(extent = {{-150, -40}, {150, -80}}, textString = "R=%R"), Line(visible = useHeatPort, points = {{0, -100}, {0, -30}}, color = {127, 0, 0}, pattern = LinePattern.Dot), Text(extent = {{-150, 90}, {150, 50}}, textString = "%name", lineColor = {0, 0, 255})}));
    end ourResistor;

    model scheme_2
      Modelica.Electrical.Analog.Basic.Resistor resistor(R = 1000) annotation(
        Placement(visible = true, transformation(origin = {22, 24}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Electrical.Analog.Basic.Resistor resistor1(R = 2000) annotation(
        Placement(visible = true, transformation(origin = {-50, -8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Electrical.Analog.Basic.Capacitor capacitor(C = 0.0000000001) annotation(
        Placement(visible = true, transformation(origin = {22, 66}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Electrical.Analog.Basic.Capacitor capacitor1(C = 0.000000001) annotation(
        Placement(visible = true, transformation(origin = {84, 8}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
      Modelica.Electrical.Analog.Sources.SineVoltage sineVoltage(V = 220, freqHz = 50) annotation(
        Placement(visible = true, transformation(origin = {-84, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      labs_OOM.lab_3.ourResistor ourResistor(R = 1000) annotation(
        Placement(visible = true, transformation(origin = {-42, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Electrical.Analog.Basic.Ground ground annotation(
        Placement(visible = true, transformation(origin = {-82, -44}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(sineVoltage.n, ourResistor.p) annotation(
        Line(points = {{-84, 32}, {-52, 32}, {-52, 50}}, color = {0, 0, 255}));
      connect(ourResistor.n, capacitor.p) annotation(
        Line(points = {{-32, 50}, {12, 50}, {12, 66}}, color = {0, 0, 255}));
      connect(ourResistor.n, resistor.p) annotation(
        Line(points = {{-32, 50}, {12, 50}, {12, 24}}, color = {0, 0, 255}));
      connect(capacitor.n, capacitor1.p) annotation(
        Line(points = {{32, 66}, {84, 66}, {84, 18}}, color = {0, 0, 255}));
      connect(resistor.n, capacitor1.p) annotation(
        Line(points = {{32, 24}, {84, 24}, {84, 18}}, color = {0, 0, 255}));
      connect(resistor1.n, capacitor1.p) annotation(
        Line(points = {{-40, -8}, {84, -8}, {84, 18}}, color = {0, 0, 255}));
      connect(resistor1.p, sineVoltage.p) annotation(
        Line(points = {{-60, -8}, {-84, -8}, {-84, 12}}, color = {0, 0, 255}));
      connect(resistor1.p, ground.p) annotation(
        Line(points = {{-60, -8}, {-82, -8}, {-82, -34}, {-82, -34}}, color = {0, 0, 255}));
    end scheme_2;

    model scheme_3
      Modelica.Electrical.Analog.Sources.SineVoltage sineVoltage(V = 220, freqHz = 50) annotation(
        Placement(visible = true, transformation(origin = {-82, 6}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Modelica.Electrical.Analog.Basic.Capacitor capacitor(C = 0.000001) annotation(
        Placement(visible = true, transformation(origin = {2, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Electrical.Analog.Basic.Inductor inductor(L = 50) annotation(
        Placement(visible = true, transformation(origin = {2, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Electrical.Analog.Basic.Resistor resistor(R = 1000) annotation(
        Placement(visible = true, transformation(origin = {44, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Electrical.Analog.Basic.Resistor resistor1(R = 1000) annotation(
        Placement(visible = true, transformation(origin = {44, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Electrical.Analog.Basic.Resistor resistor2(R = 3000) annotation(
        Placement(visible = true, transformation(origin = {62, -28}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Electrical.Analog.Basic.Ground ground annotation(
        Placement(visible = true, transformation(origin = {-82, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      lab_3.ourResistor ourResistor(R = 1000) annotation(
        Placement(visible = true, transformation(origin = {-60, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(capacitor.n, resistor.p) annotation(
        Line(points = {{12, 60}, {34, 60}, {34, 60}, {34, 60}}, color = {0, 0, 255}));
      connect(inductor.n, resistor1.p) annotation(
        Line(points = {{12, 30}, {34, 30}, {34, 30}, {34, 30}}, color = {0, 0, 255}));
      connect(resistor.n, resistor2.n) annotation(
        Line(points = {{54, 60}, {72, 60}, {72, -28}}, color = {0, 0, 255}));
      connect(resistor1.n, resistor2.n) annotation(
        Line(points = {{54, 30}, {72, 30}, {72, -28}}, color = {0, 0, 255}));
      connect(resistor2.p, sineVoltage.p) annotation(
        Line(points = {{52, -28}, {-82, -28}, {-82, -4}, {-82, -4}}, color = {0, 0, 255}));
      connect(resistor2.p, ground.p) annotation(
        Line(points = {{52, -28}, {-82, -28}, {-82, -50}}, color = {0, 0, 255}));
      connect(sineVoltage.n, ourResistor.p) annotation(
        Line(points = {{-82, 16}, {-70, 16}, {-70, 60}, {-70, 60}}, color = {0, 0, 255}));
      connect(ourResistor.n, capacitor.p) annotation(
        Line(points = {{-50, 60}, {-8, 60}, {-8, 60}, {-8, 60}}, color = {0, 0, 255}));
      connect(ourResistor.n, inductor.p) annotation(
        Line(points = {{-50, 60}, {-8, 60}, {-8, 30}, {-8, 30}}, color = {0, 0, 255}));
    end scheme_3;
  end lab_3;

  package TransportSystem
    model Tolpa
      Real V(start = 10);
      ValInput Vvh;
      ValInput Vvih;
      ValOutput V_out;
    equation
      der(V) = Vvh.Val - Vvih.Val;
      V_out.Val = V;
    end Tolpa;

    connector ValInput
    input Real Val;
    end ValInput;

    connector ValOutput
    output Real Val;
    end ValOutput;

    model Station
      Real Vvih;
      Real IsFull;
      parameter Real C = 20;
      parameter Real D = 0.2;
      ValInput V;
      ValOutput IsFull_Out;
      ValOutput Vvih_Out;
    equation

      if V.Val>C then
        IsFull = 1;
      else
        IsFull = 0;
      end if;
      if V.Val>0 then
        Vvih = D;
      else
        Vvih = 0;
      end if;
      IsFull_Out.Val = IsFull;
      Vvih_Out.Val = Vvih;
    end Station;

    model HumanStream
      Real L;
      ValInput IsFull;
      ValOutput L_Out;
    equation
      L=(1-IsFull.Val)*(0.2+0.1*sin(0.005*time));
      L_Out.Val = L;
    end HumanStream;

    model Train
      Real P(start = 0);
      Real flag(start = 0);
      parameter Real dt = 180;
      parameter Real T = 120;
      parameter Real Tpos = 15;
      parameter Real N = 5;
      parameter Real V_posadki = 0.5;
      
      ValInput V;
      ValOutput DP;
    
    equation
      if sin(3.14*time/dt) < 0 and sin(3.14*time/dt+Tpos) > 0 then
        flag = 1;
      else
        flag = 0;
      end if;
      if flag < 0.5 then
        der(P) = 0;
      else
        der(P) = V_posadki;
      end if;
        
      DP.Val = der(P);  
    end Train;

    model TwoStations
    
      Station SIn(D=0);
      Station SOut;
      Train R2DNo_6556;
      Tolpa TIn;
      Tolpa TOut(V(start=0));
      HumanStream ToHome;
      
    equation
  connect(SIn.IsFull_Out,ToHome.IsFull);
      connect(ToHome.L_Out,TIn.Vvh);
      connect(TIn.V_out,SIn.V);
      connect(TIn.V_out,R2DNo_6556.V);
      connect(R2DNo_6556.DP,TIn.Vvih);
      connect(R2DNo_6556.DP,TOut.Vvh);
      connect(TOut.V_out,SOut.V);
      connect(SOut.Vvih_Out,TOut.Vvih);
    end TwoStations;
  end TransportSystem;

  package Food_Delivery
    connector Input
      input Real Value;
    end Input;

    connector Output
    output Real Value;
    end Output;

    model Client
    Real num_cl(start = 0);
    Output n;
    equation
num_cl = (sin(time*0.005 + 3.14)+1.5)*100;
    n.Value = div(num_cl,1);
    end Client;

    model Cook
    Real isWorking(start = 0);
    Real t_start;
    parameter Real t = 30;
    Input req_cnt;
    Output isWorking_out;
    Output ckreq_done;
    equation
if(isWorking == 0) then
      if(req_cnt.Value > 0.5) then
        ckreq_done.Value = 0;
        t_start = time;
        isWorking = 1;
      else
        ckreq_done.Value = 0;
        t_start = 0;
        isWorking = 0;
      end if;
    else  
      if(time - (t_start+t) > 0) then
        der(t_start) = 0; 
        isWorking = 0;
        ckreq_done.Value = 1;
      else
        isWorking = 1;
        der(t_start) = 0;
        ckreq_done.Value = 0;
      end if;
    end if;
    isWorking_out.Value = isWorking;
    end Cook;

    model Courier
    Real isWorking(start = 0);
    Real t_start;
    parameter Real t_tocl = 40;
    parameter Real t_back = 40;
    Input req_cnt;
    Output isWorking_out;
    equation
    if(isWorking == 0) then
      if(req_cnt.Value > 0.5) then
        t_start = time;
        isWorking = 1;
      else
        t_start = 0;
        isWorking = 0;
      end if;
    else  
      if(time - (t_start+t_tocl) > 0) then
        t_start = time; 
        isWorking = 2;
      else
        isWorking = 1;
        der(t_start) = 0;
      end if; 
    end if;
    if(isWorking == 2 and (time - (t_start+t_back) > 0)) then
      der(t_start) = 0;
      isWorking = 0;
    else
      isWorking = 1;
      der(t_start) = 0;
    end if;
    isWorking_out.Value = isWorking;
    end Courier;

    model Cafe
    Input req;
    parameter Integer n_cook = 10;
    parameter Integer n_courier = 10;
    Real req_cnt;
    Real req_tmp;
    Input isWorking_ck[n_cook];
    Input ckreq_done[n_cook];
    Output req_tocook[n_cook];
    Input isWorking_cr[n_courier];
    Output req_tocourier[n_courier];
    equation
    req_cnt = req.Value;
    req_tmp = req.Value;
    for i in 1:n_cook loop
      if(isWorking_ck[i].Value == 0) then
        req_tocook[i].Value = 1;
        req_cnt = req_tmp - 1;
        req_tmp = req_cnt;
      else
        req_tocook[i].Value = 0;
        der(req_cnt) = 0;
        der(req_tmp) = 0;  
      end if;
    end for;
    for i in 1:n_courier loop
      if(isWorking_cr[i].Value == 0) then
        for k in 1:n_cook loop
          if(ckreq_done[k].Value == 1) then
            req_tocourier[i].Value = 1;
          else
            req_tocourier[i].Value = 0;
          end if;
        end for;
      else
        req_tocourier[i].Value = 0;
      end if;
    end for;
    end Cafe;

    model System
    Client client;
    Cook cook1;
    Cook cook2;
    Cook cook3;
    Cook cook4;
    Cook cook5;
    Cook cook6;
    Cook cook7;
    Cook cook8;
    Cook cook9;
    Cook cook10;
    Courier courier1;
    Courier courier2;
    Courier courier3;
    Courier courier4;
    Courier courier5;
    Courier courier6;
    Courier courier7;
    Courier courier8;
    Courier courier9;
    Courier courier10;
    Cafe cafe;
    equation
    connect(client.n, cafe.req);
    connect(cafe.req_tocook[1], cook1.req_cnt);
    connect(cafe.req_tocook[2], cook2.req_cnt);
    connect(cafe.req_tocook[3], cook3.req_cnt);
    connect(cafe.req_tocook[4], cook4.req_cnt);
    connect(cafe.req_tocook[5], cook5.req_cnt);
    connect(cafe.req_tocook[6], cook6.req_cnt);
    connect(cafe.req_tocook[7], cook7.req_cnt);
    connect(cafe.req_tocook[8], cook8.req_cnt);
    connect(cafe.req_tocook[9], cook9.req_cnt);
    connect(cafe.req_tocook[10], cook10.req_cnt);
    connect(cook1.isWorking_out, cafe.isWorking_ck[1]);
    connect(cook2.isWorking_out, cafe.isWorking_ck[2]);
    connect(cook3.isWorking_out, cafe.isWorking_ck[3]);
    connect(cook4.isWorking_out, cafe.isWorking_ck[4]);
    connect(cook5.isWorking_out, cafe.isWorking_ck[5]);
    connect(cook6.isWorking_out, cafe.isWorking_ck[6]);
    connect(cook7.isWorking_out, cafe.isWorking_ck[7]);
    connect(cook8.isWorking_out, cafe.isWorking_ck[8]);
    connect(cook9.isWorking_out, cafe.isWorking_ck[9]);
    connect(cook10.isWorking_out, cafe.isWorking_ck[10]);
    connect(cook1.ckreq_done, cafe.ckreq_done[1]);
    connect(cook2.ckreq_done, cafe.ckreq_done[2]);
    connect(cook3.ckreq_done, cafe.ckreq_done[3]);
    connect(cook4.ckreq_done, cafe.ckreq_done[4]);
    connect(cook5.ckreq_done, cafe.ckreq_done[5]);
    connect(cook6.ckreq_done, cafe.ckreq_done[6]);
    connect(cook7.ckreq_done, cafe.ckreq_done[7]);
    connect(cook8.ckreq_done, cafe.ckreq_done[8]);
    connect(cook9.ckreq_done, cafe.ckreq_done[9]);
    connect(cook10.ckreq_done, cafe.ckreq_done[10]);
    connect(cafe.req_tocourier[1], courier1.req_cnt);
    connect(cafe.req_tocourier[2], courier2.req_cnt);
    connect(cafe.req_tocourier[3], courier3.req_cnt);
    connect(cafe.req_tocourier[4], courier4.req_cnt);
    connect(cafe.req_tocourier[5], cook5.req_cnt);
    connect(cafe.req_tocourier[6], cook6.req_cnt);
    connect(cafe.req_tocourier[7], cook7.req_cnt);
    connect(cafe.req_tocourier[8], cook8.req_cnt);
    connect(cafe.req_tocourier[9], cook9.req_cnt);
    connect(cafe.req_tocourier[10], cook10.req_cnt);
    end System;
  end Food_Delivery;

  package Cafe
    connector Input
      input Integer Value;
    end Input;

    connector Output
    output Integer Value;
    end Output;

    model Client
    Real num_cl(start = 0);
    Output cl_req;
    equation
    if((sin(time*0.005 + 3.14)) > 0) then
      der(num_cl) = sin(time*0.005 + 3.14);
    else
      der(num_cl) = -sin(time*0.005 + 3.14);
    end if;
    cl_req.Value = div(num_cl,1);
    end Client;

    model Cook
    Input cl_req;
    Real req_done;
    Real working_flag(start = 0);
    Real N_req_done;
    Output N_req_done_out;
    Real v_cook;
    equation
    if(mod(N_req_done,15) == 0) then
      v_cook = (1 - sin(time*0.5))/1.5;
    else
      v_cook = 1 - sin(time*0.5);
    end if;
    if(working_flag == 0) then
      if(req_done == cl_req.Value) then
        working_flag = 0;
        der(req_done) = 0;
      else
        working_flag = 1;
        der(req_done) = v_cook;
      end if;
    else
      if(req_done < cl_req.Value) then
        der(req_done) = v_cook;
        working_flag = 1;
      else
        der(req_done) = 0;
        working_flag = 0;
      end if;
    end if;
    N_req_done = div(req_done,1);
    N_req_done_out.Value = N_req_done;
    end Cook;

    model System
    Client client;
    Cook cook;
    Courier courier;
    equation
    connect(client.cl_req, cook.cl_req);
    connect(cook.N_req_done_out, courier.done_cook);
    end System;

    connector Input_array
    input Integer Value[2];
    end Input_array;

    connector Output_array
    output Integer Value[2];
    end Output_array;

    model Courier
    Input done_cook;
    Real dvr_done;
    Real working_flag;
    Integer N_req_done;
    Real v_courier;
    equation
    if(mod(N_req_done,5) == 0) then
      v_courier = (1 - cos(time))/1.5;
    else
      v_courier = (1 - cos(time));
    end if;
if(working_flag == 0) then
      if(dvr_done == done_cook.Value) then
        working_flag = 0;
        der(dvr_done) = 0;
      else
        working_flag = 1;
        der(dvr_done) = v_courier;
      end if;
    else
      if(dvr_done < done_cook.Value) then
        der(dvr_done) = v_courier;
        working_flag = 1;
      else
        der(dvr_done) = 0;
        working_flag = 0;
      end if;
    end if;
    N_req_done = div(dvr_done,1);
    end Courier;
  end Cafe;

  model lab_1
  parameter Integer n = 10;
    Real x[n];
    Real y[n];
  initial equation
    for i in 1:n loop
      x[i] = 0.5 * i;
      y[i] = i / n;
    end for;
  equation
    for i in 1:n loop
      der(x[i]) = -y[i];
      der(y[i]) = x[i] - y[i];
    end for;
  end lab_1;
  annotation(
    uses(Modelica(version = "3.2.3")));
end labs_OOM;
