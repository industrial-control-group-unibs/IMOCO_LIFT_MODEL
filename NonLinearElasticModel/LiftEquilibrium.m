function [x_eq,u_eq] = LiftEquilibrium(BuildingHeight,LinearStiffness,Mc,Mw,Rp,dc,dw,g,gearbox,min_length,mu,state9)
%LiftEquilibrium
%    [X_EQ,U_EQ] = LiftEquilibrium(BuildingHeight,LinearStiffness,Mc,Mw,Rp,DC,DW,G,GEARBOX,MIN_LENGTH,MU,STATE9)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    19-Oct-2023 10:13:21

%Version: 1.0
mt1 = [((dw.*min_length.*2.0-BuildingHeight.*LinearStiffness.*2.0+g.*min_length.^2.*mu+Mw.*g.*min_length.*2.0+LinearStiffness.*Rp.*gearbox.*state9.*2.0+Rp.*dw.*gearbox.*state9.*2.0+Mw.*Rp.*g.*gearbox.*state9.*2.0+Rp.^2.*g.*gearbox.^2.*mu.*state9.^2+Rp.*g.*gearbox.*min_length.*mu.*state9.*2.0).*(-1.0./2.0))./LinearStiffness;0.0;((-LinearStiffness.*min_length+dw.*min_length-BuildingHeight.*LinearStiffness.*2.0+g.*min_length.^2.*mu+Mw.*g.*min_length+LinearStiffness.*Rp.*gearbox.*state9+Rp.*dw.*gearbox.*state9+Mw.*Rp.*g.*gearbox.*state9+Rp.^2.*g.*gearbox.^2.*mu.*state9.^2+Rp.*g.*gearbox.*min_length.*mu.*state9.*2.0).*(-1.0./2.0))./LinearStiffness;0.0];
mt2 = [((-LinearStiffness.*min_length+dc.*min_length-BuildingHeight.*LinearStiffness+g.*min_length.^2.*mu+Mc.*g.*min_length-LinearStiffness.*Rp.*gearbox.*state9+BuildingHeight.*g.*min_length.*mu+Rp.*dc.*gearbox.*state9+Mc.*Rp.*g.*gearbox.*state9-Rp.^2.*g.*gearbox.^2.*mu.*state9.^2+BuildingHeight.*Rp.*g.*gearbox.*mu.*state9).*(-1.0./2.0))./LinearStiffness;0.0;((dc.*min_length.*2.0+g.*min_length.^2.*mu+Mc.*g.*min_length.*2.0-LinearStiffness.*Rp.*gearbox.*state9.*2.0+BuildingHeight.*g.*min_length.*mu+Rp.*dc.*gearbox.*state9.*2.0+Mc.*Rp.*g.*gearbox.*state9.*2.0-Rp.^2.*g.*gearbox.^2.*mu.*state9.^2+BuildingHeight.*Rp.*g.*gearbox.*mu.*state9).*(-1.0./2.0))./LinearStiffness;0.0;state9;0.0];
x_eq = [mt1;mt2];
if nargout > 1
    u_eq = Rp.*gearbox.*(dc-dw+Mc.*g-Mw.*g+BuildingHeight.*g.*mu-Rp.*g.*gearbox.*mu.*state9.*2.0);
end
