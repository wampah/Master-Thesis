import os
import numpy as np

def generate_mujoco_xml(timestep,o1x,o1y,o2x,o2y,L_p1,L_p2,L_d1,L_d2,link_width,link_height,link_separation,rotor1_D,rotor2_D,rotor1_H,rotor2_H,
                        max_torque,q0,
                        inertia_rotor1,inertia_rotor2,inertia_p1,inertia_p2,inertia_d1,inertia_d2,
                        com_offset_p1,com_offset_p2,com_offset_d1,com_offset_d2,
                        mass_rotor1,mass_rotor2,mass_p1,mass_p2,mass_d1,mass_d2,
                        damping_bp1,damping_bp2,damping_p1d1,damping_p2d2,damping_d1d2,
                        dryfrict_bp1,dryfrict_bp2,dryfrict_p1d1,dryfrict_p2d2,dryfrict_d1d2
                        ):
    xml_template = f"""
<mujoco model="parallel_5_bar_mechanism">
    <compiler angle="degree"/>
    <option gravity="0 0 -9.81" integrator="RK4" timestep="{timestep}" />
    <asset>
        <texture type="skybox" builtin="gradient" rgb1=".4 .5 .6" rgb2="0 0 0" width="100" height="100"/>
        <texture name="grid" type="2d" builtin="checker" rgb1=".1 .2 .3" rgb2=".2 .3 .4" width="300" height="300"/>
        <material name="grid" texture="grid" texrepeat="8 8" reflectance=".2"/>
    </asset>

    <worldbody>
        <light cutoff="100" diffuse="1 1 1" dir="-0 0 -1.3" directional="true" exponent="1" pos="0 0 1.3" specular=".1 .1 .1"/>
        <geom name="worldbody" size="1 1 .01" pos="0 0 -.2" type="plane" material="grid"/>
        <body name="base" pos="0 0 0">
            <body name="proximal1" pos="{o1x} {o1y} {1.5*link_height+1.5*link_separation}" euler="0 0 {np.rad2deg(q0[0])}">
                <inertial pos='{L_p1/2+com_offset_p1} 0 0' mass='{mass_p1}' diaginertia='{inertia_p1} {inertia_p1} {inertia_p1}'/>
                <body name="rotor1" pos="0 0 {rotor1_H/2+link_height/2}"> 
                    <geom type="cylinder" size="{rotor1_D/2} {rotor1_H/2}" rgba="0 0 0 1"/>
                    <inertial pos="0 0 0" mass="{mass_rotor1}" diaginertia='{inertia_rotor1} {inertia_rotor1} {inertia_rotor1}' />
                </body>
                <joint name="joint1" type="hinge" axis="0 0 1" pos="0 0 0"   damping="{damping_bp1}" frictionloss="{dryfrict_bp1}" stiffness="0" ref="{np.rad2deg(q0[0])}"/>
                <geom name="proximal1" type="box" size="{(L_p1)/2} {link_width/2} {link_height/2}" pos="{L_p1/2} 0 0" rgba="1 0 0 1"/>
                <body name="distal1" pos="{L_p1} 0 {-link_height-link_separation}" euler="0 0 {np.rad2deg(q0[1]-q0[0])}">
                    <inertial pos='{L_d1/2+com_offset_d1} 0 0' mass='{mass_d1}' diaginertia='{inertia_d1} {inertia_d1} {inertia_d1}'/>
                    <joint name="joint3" type="hinge" axis="0 0 1" pos="0 0 0"  damping="{damping_p1d1}" frictionloss="{dryfrict_p1d1}" stiffness="0" ref="{np.rad2deg(q0[1]-q0[0])}"/>
                    <geom name="distal1" type="box" size="{L_d1/2} {link_width/2} {link_height/2}" pos="{L_d1/2} 0 0" rgba="0 1 0 1"/>

                </body>
            </body>
            <body name="proximal2" pos="{o2x} {o2y} {-1.5*link_height-1.5*link_separation}" euler="0 0 {np.rad2deg(q0[3])}">
                <inertial pos='{-L_p2/2+com_offset_p2} 0 0' mass='{mass_p2}' diaginertia='{inertia_p2} {inertia_p2} {inertia_p2}'/>
                <body name="rotor2" pos="0 0 -{rotor2_H/2+link_height/2}"> 
                    <geom type="cylinder" size="{rotor2_D/2} {rotor2_H/2}" rgba="0 0 0 1"/>
                    <inertial pos="0 0 0" mass="{mass_rotor2}" diaginertia='{inertia_rotor2} {inertia_rotor2} {inertia_rotor2}' />
                </body>
                <joint name="joint2" type="hinge" axis="0 0 1" pos="0 0 0"  damping="{damping_bp2}" frictionloss="{dryfrict_bp2}" stiffness="0"  ref="{np.rad2deg(q0[3])}"/>
                <geom name="proximal2" type="box" size="{L_p2/2} {(link_width)/2} {link_height/2}" pos="{-L_p2/2} 0 0" rgba="1 0 0 1"/>
                <body name="distal2" pos="{-L_p2} 0 {link_height+link_separation}" euler="0 0 {np.rad2deg(q0[2]-q0[3])}">
                    <inertial pos='{-L_d2/2+com_offset_d2} 0 0' mass='{mass_d2}' diaginertia='{inertia_d2} {inertia_d2} {inertia_d2}'/>
                    <joint name="joint4" type="hinge" axis="0 0 1" pos="0 0 0"  damping="{damping_p2d2}" frictionloss="{dryfrict_p2d2}" stiffness="0" ref="{np.rad2deg(q0[2]-q0[3])}"/>
                    <geom name="distal2" type="box" size="{L_d2/2} {link_width/2} {link_height/2}" pos="{-L_d2/2} 0 0" rgba="0 1 0 1"/>
                    <body name="end_effector" pos="{-L_d2} 0 0">
                        <joint name="joint5" type="hinge" axis="0 0 1" pos="0 0 0"   damping="{damping_d1d2}" frictionloss="{dryfrict_d1d2}" stiffness="0" ref="{np.rad2deg(q0[2]-q0[1])}"/>
					    <geom contype="0" name="end_effector" pos="0 0 0" rgba="0.0 0.8 0.6 1" size=".001" type="sphere"/>
                        <inertial pos="0 0 0" mass="0.000001" diaginertia='0.00000001 0.00000001 0.00000001' />
				    </body>
                </body>
            </body>
        </body>
        <body name="target" pos=".125 .125 0">
			<joint armature="0" axis="1 0 0" damping="0" limited="true" name="target_x" pos="0 0 0" range="-1 1" ref=".125" stiffness="0" type="slide"/>
			<joint armature="0" axis="0 1 0" damping="0" limited="true" name="target_y" pos="0 0 0" range="-1 1" ref=".125" stiffness="0" type="slide"/>
			<geom conaffinity="0" contype="0" name="target" pos="0 0 0" rgba="0.9 0.2 0.2 1" size="0.01" type="sphere"/>
	    </body>
    </worldbody>
    <equality>
		<weld anchor="0 0 0" active="true" torquescale="1000" body1="distal1" body2="end_effector" name="equality_constraint"/>
	</equality>
    <sensor>
		<framepos objtype="body" objname="end_effector"/>
		<framelinvel objtype="body" objname="end_effector"/>
	</sensor>
    <actuator>
        <!position joint="joint1" ctrllimited="true"  ctrlrange="0 3.14" kp="3" kv="0.1"/>
        <!position joint="joint2" ctrllimited="true"  ctrlrange="-3.14 0" kp="3" kv="0.1"/>
        <motor joint="joint1" ctrllimited="true"  ctrlrange="-{max_torque} {max_torque}"/>
        <motor joint="joint2" ctrllimited="true"  ctrlrange="-{max_torque} {max_torque}"/>
    </actuator>
</mujoco>
"""
    return xml_template

def generate_realistic_mujoco_xml(timestep,o1x,o1y,o2x,o2y,L_p1,L_p2,L_d1,L_d2,link_width,link_height,link_separation,rotor1_D,rotor2_D,rotor1_H,rotor2_H,
                        max_torque,q0,
                        inertia_rotor1,inertia_rotor2,inertia_p1,inertia_p2,inertia_d1,inertia_d2,
                        com_offset_p1,com_offset_p2,com_offset_d1,com_offset_d2,
                        mass_rotor1,mass_rotor2,mass_p1,mass_p2,mass_d1,mass_d2,
                        damping_bp1,damping_bp2,damping_p1d1,damping_p2d2,damping_d1d2,
                        dryfrict_bp1,dryfrict_bp2,dryfrict_p1d1,dryfrict_p2d2,dryfrict_d1d2
                        ):
    xml_template = f"""
<mujoco model="parallel_5_bar_mechanism">
    <compiler meshdir="./mesh" angle="degree"/>
    <option gravity="0 0 -9.81" integrator="RK4" timestep="{timestep}"/>
    <asset>
    
        <texture type="skybox" builtin="gradient" rgb1=".4 .5 .6" rgb2="0 0 0" width="100" height="100"/>

        <texture name="grid" type="2d" builtin="checker" rgb1=".1 .2 .3" rgb2=".2 .3 .4" width="300" height="300"/>
        
        <material name="grid" texture="grid" texrepeat="8 8" reflectance=".2"/>
        
        <mesh name="Proximal_Link_1" file="Proximal Link 1.stl" scale="0.001 0.001 0.001"/>

        <mesh name="Proximal_Link_2" file="Proximal Link 2.stl" scale="0.001 0.001 0.001"/>

        <mesh name="Distal_Link_1" file="Distal Link 1.stl" scale="0.001 0.001 0.001"/>

        <mesh name="Distal_Link_2" file="Distal Link 2.stl" scale="0.001 0.001 0.001"/>
        
        <material name="aluminum_material" rgba="0.8 0.8 0.8 1" specular="0.8" shininess="0.9" reflectance="0.3"/>
        
     </asset>

    <worldbody>
        <light cutoff="100" diffuse="1 1 1" dir="-0 0 -1.3" directional="true" exponent="1" pos="0 0 1.3" specular=".1 .1 .1"/>
        <geom name="worldbody" size="1 1 .01" pos="0 0 -.2" type="plane" material="grid"/>
        <body name="base" pos="0 0 0">
            <body name="proximal1" pos="{o1x} {o1y} {1.5*link_height+1.5*link_separation}" euler="0 0 {np.rad2deg(q0[0])}">
                <inertial pos='{L_p1/2+com_offset_p1} 0 0' mass='{mass_p1}' diaginertia='{inertia_p1} {inertia_p1} {inertia_p1}'/>
                <body name="rotor1" pos="0 0 {rotor1_H/2+link_height/2}"> 
                    <geom type="cylinder" size="{rotor1_D/2} {rotor1_H/2}" rgba="0 0 0 1"/>
                    <inertial pos="0 0 0" mass="{mass_rotor1}" diaginertia='{inertia_rotor1} {inertia_rotor1} {inertia_rotor1}' />
                </body>
                <joint name="joint1" type="hinge" axis="0 0 1" pos="0 0 0"   damping="{damping_bp1}" frictionloss="{dryfrict_bp1}" stiffness="0" ref="{np.rad2deg(q0[0])}"/>
                <geom name="proximal1" type="mesh" mesh="Proximal_Link_1" condim="1" material="aluminum_material"/>
                <body name="distal1" pos="{L_p1} 0 {-link_height-link_separation}" euler="0 0 {np.rad2deg(q0[1]-q0[0])}">
                    <inertial pos='{L_d1/2+com_offset_d1} 0 0' mass='{mass_d1}' diaginertia='{inertia_d1} {inertia_d1} {inertia_d1}'/>
                    <joint name="joint3" type="hinge" axis="0 0 1" pos="0 0 0"  damping="{damping_p1d1}" frictionloss="{dryfrict_p1d1}" stiffness="0" ref="{np.rad2deg(q0[1]-q0[0])}"/>
                    <geom name="distal1" type="mesh" mesh="Distal_Link_1" condim="1" material="aluminum_material"/>
                </body>
            </body>
            <body name="proximal2" pos="{o2x} {o2y} {-1.5*link_height-1.5*link_separation}" euler="0 0 {np.rad2deg(q0[3])}">
                <inertial pos='{-L_p2/2+com_offset_p2} 0 0' mass='{mass_p2}' diaginertia='{inertia_p2} {inertia_p2} {inertia_p2}'/>
                <body name="rotor2" pos="0 0 -{rotor2_H/2+link_height/2}"> 
                    <geom type="cylinder" size="{rotor2_D/2} {rotor2_H/2}" rgba="0 0 0 1"/>
                    <inertial pos="0 0 0" mass="{mass_rotor2}" diaginertia='{inertia_rotor2} {inertia_rotor2} {inertia_rotor2}' />
                </body>
                <joint name="joint2" type="hinge" axis="0 0 1" pos="0 0 0"  damping="{damping_bp2}" frictionloss="{dryfrict_bp2}" stiffness="0"  ref="{np.rad2deg(q0[3])}"/>
                <geom name="proximal2" type="mesh" mesh="Proximal_Link_2" condim="1" material="aluminum_material"/>
                <body name="distal2" pos="{-L_p2} 0 {link_height+link_separation}" euler="0 0 {np.rad2deg(q0[2]-q0[3])}">
                    <inertial pos='{-L_d2/2+com_offset_d2} 0 0' mass='{mass_d2}' diaginertia='{inertia_d2} {inertia_d2} {inertia_d2}'/>
                    <joint name="joint4" type="hinge" axis="0 0 1" pos="0 0 0"  damping="{damping_p2d2}" frictionloss="{dryfrict_p2d2}" stiffness="0" ref="{np.rad2deg(q0[2]-q0[3])}"/>
                    <geom name="distal2" type="mesh" mesh="Distal_Link_2" condim="1" material="aluminum_material"/>
                    <body name="end_effector" pos="{-L_d2} 0 0">
                        <joint name="joint5" type="hinge" axis="0 0 1" pos="0 0 0"   damping="{damping_d1d2}" frictionloss="{dryfrict_d1d2}" stiffness="0" ref="{np.rad2deg(q0[2]-q0[1])}"/>
					    <geom contype="0" name="end_effector" pos="0 0 0" rgba="0.0 0.8 0.6 1" size=".001" type="sphere"/>
                        <inertial pos="0 0 0" mass="0.000001" diaginertia='0.00000001 0.00000001 0.00000001' />
				    </body>
                </body>
            </body>
        </body>
        <body name="target" pos=".125 .125 0">
			<joint armature="0" axis="1 0 0" damping="0" limited="true" name="target_x" pos="0 0 0" range="-1 1" ref=".125" stiffness="0" type="slide"/>
			<joint armature="0" axis="0 1 0" damping="0" limited="true" name="target_y" pos="0 0 0" range="-1 1" ref=".125" stiffness="0" type="slide"/>
			<geom conaffinity="0" contype="0" name="target" pos="0 0 0" rgba="0.9 0.2 0.2 1" size="0.01" type="sphere"/>
	    </body>
    </worldbody>
    <equality>
		<weld anchor="0 0 0"  active="true" torquescale="1000" body1="distal1" body2="end_effector" name="equality_constraint "/>
	</equality>
    <sensor>
		<framepos objtype="body" objname="end_effector"/>
		<framelinvel objtype="body" objname="end_effector"/>
	</sensor>
    <actuator>
        <!position joint="joint1" ctrllimited="true"  ctrlrange="0 3.14" kp="10" kv="1"/>
        <!position joint="joint2" ctrllimited="true"  ctrlrange="-3.14 0" kp="10" kv="1"/>
        <motor joint="joint1" ctrllimited="true"  ctrlrange="-{max_torque} {max_torque}"/>
        <motor joint="joint2" ctrllimited="true"  ctrlrange="-{max_torque} {max_torque}"/>
    </actuator>
</mujoco>
"""
    return xml_template

timestep=0.01
scale=0.5
o1x=-0.075*scale
o1y=0*scale
o2x=0.075*scale
o2y=0*scale
L_p1=0.2*scale
L_p2=0.2*scale
L_d1=0.4*scale
L_d2=0.4*scale
link_width=0.02
link_height=0.026
link_separation=0.003
max_torque=1.6

rotor1_D=89e-3# From Motor Spec
rotor2_D=rotor1_D
rotor1_H=28e-3# From Motor Spec
rotor2_H=rotor1_H

q0=[2.356194490192345,
    0.999107158546108,
    -0.999107158546108,
    -2.356194490192345]



inertia_rotor1=0.0036246659837414308
inertia_rotor2=0.0036246659837414308
inertia_p1=0.003404427722363115
inertia_p2=0.003404427722363115
inertia_d1=0.004724720400236029
inertia_d2=0.004724720400236029
com_offset_p1=-0.008133117577778232
com_offset_p2=0.008133117577778232
com_offset_d1=0.01
com_offset_d2=-0.01
mass_rotor1=0.65
mass_rotor2=0.65
mass_p1=0.1472
mass_p2=0.1472
mass_d1=0.1934
mass_d2=0.1934
damping_bp1=0.01
damping_bp2=0.00690291315996407
damping_p1d1=0.01
damping_p2d2=0.01
dryfrict_bp1=0.07378660332065916
dryfrict_bp2=0.08236076287466008
dryfrict_p1d1=0.017929769090750527
dryfrict_p2d2=0.017929769090750527
damping_d1d2=0.1
dryfrict_d1d2=0.1

inertia_rotor1=0.0003271 # From Motor Spec
inertia_rotor2=inertia_rotor1

mass_rotor1=0.65
mass_rotor2=mass_rotor1

com_offset_p1=.05
com_offset_p2=-com_offset_p1
com_offset_d1=.05
com_offset_d2=-com_offset_d1

xml_content = generate_mujoco_xml(timestep,o1x,o1y,o2x,o2y,L_p1,L_p2,L_d1,L_d2,link_width,link_height,link_separation,rotor1_D,rotor2_D,rotor1_H,rotor2_H,
                        max_torque,q0,
                        inertia_rotor1,inertia_rotor2,inertia_p1,inertia_p2,inertia_d1,inertia_d2,
                        com_offset_p1,com_offset_p2,com_offset_d1,com_offset_d2,
                        mass_rotor1,mass_rotor2,mass_p1,mass_p2,mass_d1,mass_d2,
                        damping_bp1,damping_bp2,damping_p1d1,damping_p2d2,damping_d1d2,
                        dryfrict_bp1,dryfrict_bp2,dryfrict_p1d1,dryfrict_p2d2,dryfrict_d1d2
                        )

xml_realistic = generate_realistic_mujoco_xml(timestep,o1x,o1y,o2x,o2y,L_p1,L_p2,L_d1,L_d2,link_width,link_height,link_separation,rotor1_D,rotor2_D,rotor1_H,rotor2_H,
                        max_torque,q0,
                        inertia_rotor1,inertia_rotor2,inertia_p1,inertia_p2,inertia_d1,inertia_d2,
                        com_offset_p1,com_offset_p2,com_offset_d1,com_offset_d2,
                        mass_rotor1,mass_rotor2,mass_p1,mass_p2,mass_d1,mass_d2,
                        damping_bp1,damping_bp2,damping_p1d1,damping_p2d2,damping_d1d2,
                        dryfrict_bp1,dryfrict_bp2,dryfrict_p1d1,dryfrict_p2d2,dryfrict_d1d2
                        )

# Save to a file
with open(os.path.join(os.path.dirname(__file__), '5_bar.xml'), "w") as f:
    f.write(xml_content)

with open(os.path.join(os.path.dirname(__file__), '5_bar_realistic.xml'), "w") as f:
    f.write(xml_realistic)

print("MuJoCo XML file generated successfully!")
