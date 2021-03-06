function out1 = iiwa_FK_symbolic_W6(theta, base_T)
%IIWA_FK_SYMBOLIC_W6
%    OUT1 = IIWA_FK_SYMBOLIC_W6(BT1_1,BT1_2,BT1_3,BT1_4,BT2_1,BT2_2,BT2_3,BT2_4,BT3_1,BT3_2,BT3_3,BT3_4,BT4_1,BT4_2,BT4_3,BT4_4,TH1,TH2,TH3,TH4,TH5)

%    This function was generated by the Symbolic Math Toolbox version 8.0.
%    18-Jul-2018 20:57:49
BT1_1=base_T(1,1);
BT1_2=base_T(1,2);
BT1_3=base_T(1,3);
BT1_4=base_T(1,4);
BT2_1=base_T(2,1);
BT2_2=base_T(2,2);
BT2_3=base_T(2,3);
BT2_4=base_T(2,4);
BT3_1=base_T(3,1);
BT3_2=base_T(3,2);
BT3_3=base_T(3,3);
BT3_4=base_T(3,4);
BT4_1=0;
BT4_2=0;
BT4_3=0;
BT4_4=1;

th1 = theta(1);
th2 = theta(2);
th3 = theta(3);
th4 = theta(4);
th5 = theta(5);
th6 = theta(6);
th7 = theta(7);


t2 = cos(th1);
t3 = sin(th1);
t4 = cos(th2);
t5 = sin(th2);
t6 = BT1_1.*t2;
t7 = BT1_2.*t3;
t8 = t6+t7;
t9 = sin(th3);
t10 = BT1_1.*t3;
t21 = BT1_2.*t2;
t11 = t10-t21;
t12 = cos(th3);
t13 = BT1_3.*t5;
t23 = t4.*t8;
t14 = t13-t23;
t15 = sin(th5);
t16 = sin(th4);
t17 = BT1_3.*t4;
t18 = t5.*t8;
t19 = t17+t18;
t20 = cos(th4);
t22 = t9.*t11;
t24 = t12.*t14;
t25 = t22+t24;
t26 = t16.*t19-t20.*t25;
t27 = cos(th5);
t28 = t9.*t14;
t29 = t28-t11.*t12;
t30 = BT2_1.*t2;
t31 = BT2_2.*t3;
t32 = t30+t31;
t33 = BT2_1.*t3;
t41 = BT2_2.*t2;
t34 = t33-t41;
t35 = BT2_3.*t5;
t43 = t4.*t32;
t36 = t35-t43;
t37 = BT2_3.*t4;
t38 = t5.*t32;
t39 = t37+t38;
t40 = t16.*t39;
t42 = t9.*t34;
t44 = t12.*t36;
t45 = t42+t44;
t46 = t40-t20.*t45;
t47 = t9.*t36;
t48 = t47-t12.*t34;
t49 = BT3_1.*t2;
t50 = BT3_2.*t3;
t51 = t49+t50;
t52 = BT3_1.*t3;
t60 = BT3_2.*t2;
t53 = t52-t60;
t54 = BT3_3.*t5;
t62 = t4.*t51;
t55 = t54-t62;
t56 = BT3_3.*t4;
t57 = t5.*t51;
t58 = t56+t57;
t59 = t16.*t58;
t61 = t9.*t53;
t63 = t12.*t55;
t64 = t61+t63;
t65 = t59-t20.*t64;
t66 = t9.*t55;
t67 = t66-t12.*t53;
t68 = BT4_1.*t2;
t69 = BT4_2.*t3;
t70 = t68+t69;
t71 = BT4_1.*t3;
t79 = BT4_2.*t2;
t72 = t71-t79;
t73 = BT4_3.*t5;
t81 = t4.*t70;
t74 = t73-t81;
t75 = BT4_3.*t4;
t76 = t5.*t70;
t77 = t75+t76;
t78 = t16.*t77;
t80 = t9.*t72;
t82 = t12.*t74;
t83 = t80+t82;
t84 = t78-t20.*t83;
t85 = t9.*t74;
t86 = t85-t12.*t72;
out1 = reshape([-t15.*t29-t26.*t27,-t15.*t48-t27.*t46,-t15.*t67-t27.*t65,-t15.*t86-t27.*t84,t15.*t26-t27.*t29,t15.*t46-t27.*t48,t15.*t65-t27.*t67,t15.*t84-t27.*t86,t16.*(t22+t24)+t19.*t20,t16.*(t42+t44)+t20.*t39,t16.*(t61+t63)+t20.*t58,t16.*(t80+t82)+t20.*t77,BT1_3.*(1.7e1./5.0e1)+BT1_4+BT1_3.*t4.*(2.0./5.0)+t16.*(t22+t24).*(2.1e1./1.0e2)+t5.*t8.*(2.0./5.0)+t19.*t20.*(2.1e1./1.0e2),BT2_3.*(1.7e1./5.0e1)+BT2_4+BT2_3.*t4.*(2.0./5.0)+t16.*(t42+t44).*(2.1e1./1.0e2)+t5.*t32.*(2.0./5.0)+t20.*t39.*(2.1e1./1.0e2),BT3_3.*(1.7e1./5.0e1)+BT3_4+BT3_3.*t4.*(2.0./5.0)+t16.*(t61+t63).*(2.1e1./1.0e2)+t5.*t51.*(2.0./5.0)+t20.*t58.*(2.1e1./1.0e2),BT4_3.*(1.7e1./5.0e1)+BT4_4+BT4_3.*t4.*(2.0./5.0)+t16.*(t80+t82).*(2.1e1./1.0e2)+t5.*t70.*(2.0./5.0)+t20.*t77.*(2.1e1./1.0e2)],[4,4]);
