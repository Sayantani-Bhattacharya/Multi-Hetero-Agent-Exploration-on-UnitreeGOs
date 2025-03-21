#include <iostream>
#include "unitree_kinematics/geometry2d.hpp"
#include "unitree_kinematics/se2d.hpp"
#include <cmath>
#include "unitree_kinematics/svg.hpp"

using unitree_kinematics::Transform2D;
using unitree_kinematics::Point2D;
using unitree_kinematics::Vector2D;
using unitree_kinematics::Twist2D;
using unitree_kinematics::normalizeVector;

int main()
{

    // Create an SVG object with width 8.5 inches and height 11 inches
    SVG svg(816, 1056);

    // Add the provided SVG code to the file
    // std::string svgCode = R"(
    // <defs>
    // <marker
    //     style="overflow:visible"
    //     id="Arrow1Sstart"
    //     refX="0.0"
    //     refY="0.0"
    //     orient="auto">
    //     <path
    //         transform="scale(0.2) translate(6,0)\"
    //         style="fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt"
    //         d="M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z "
    //         />
    //     </marker>
    // </defs>

    // <circle cx="504.2" cy="403.5" r="3" stroke="purple" fill="purple" stroke-width="1" />

    // <line x1="312.000000" x2="408.000000" y1="624.000000" y2="528.000000" stroke="purple" stroke-width="5" marker-start="url(#Arrow1Sstart)\" />

    // <g>
    // <line x1="504.000000" x2="408.000000" y1="528.000000" y2="528.000000" stroke="red" stroke-width="5" marker-start="url(#Arrow1Sstart)\" />
    // <line x1="408.000000" x2="408.000000" y1="432.000000" y2="528.000000" stroke="green" stroke-width="5" marker-start="url(#Arrow1Sstart)\" />
    // <text x="408.000000" y="528.250000">{a}</text>
    // </g>
    // )";

    std::string svgCode = R"(
    <defs>
    <marker
        style="overflow:visible"
        id="Arrow1Sstart"
        refX="0.0"
        refY="0.0"
        orient="auto">
        <path
            transform="scale(0.2) translate(6,0)\"
            style="fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt"
            d="M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z "
            />
        </marker>
    </defs>

    <g>
    
    </g>
    )";

    Transform2D Tab, Tbc;

    std::cout << "Enter transform T_{a,b}:" << std::endl;
    std::cin >> Tab;
    std::cout << "Enter transform T_{b,c}:" << std::endl;
    std::cin >> Tbc;
    Transform2D Tba{Tab.inv().translation(), Tab.inv().rotation()};
    Transform2D Tcb{Tbc.inv().translation(), Tbc.inv().rotation()};
    Transform2D Tac{(Tab*Tbc).translation(), (Tab*Tbc).rotation()};
    Transform2D Tca{Tac.inv().translation(), Tac.inv().rotation()};
    std::cout << "T_{a, b}: " << Tab << "\n";
    std::cout << "T_{b, a}: " << Tba << "\n";
    std::cout << "T_{b, c}: " << Tbc << "\n";
    std::cout << "T_{c, b}: " << Tcb << "\n";
    std::cout << "T_{a, c}: " << Tac << "\n";
    std::cout << "T_{c, a}: " << Tca << std::endl;

    Point2D origin_a{408, 728};
    Vector2D x_axis{1,0}, y_axis{0,1};
    double axis_length = 100.0;
    double unit_length = 1 * axis_length;

    // Draw Frame {a}
    svg.addLine(origin_a.x + axis_length, origin_a.y, origin_a.x, origin_a.y, "red");
    svg.addLine(origin_a.x, origin_a.y - axis_length, origin_a.x, origin_a.y, "green");
    svg.addText(origin_a.x, origin_a.y, "{a}");

    // Draw Frame {b}
    Point2D origin_b{origin_a.x + unit_length * Tab.translation().x, origin_a.y - unit_length * Tab.translation().y}; 

    svg.addLine(origin_b.x + axis_length * Tab(x_axis).x, origin_b.y - axis_length * Tab(x_axis).y, origin_b.x, origin_b.y, "red");
    svg.addLine(origin_b.x + axis_length * Tab(y_axis).x, origin_b.y - axis_length * Tab(y_axis).y, origin_b.x, origin_b.y, "green");
    svg.addText(origin_b.x, origin_b.y, "{b}");

    // Draw Frame {c}
    Point2D origin_c{origin_a.x + unit_length * Tac.translation().x, origin_a.y - unit_length * Tac.translation().y}; 

    svg.addLine(origin_c.x + axis_length * Tac(x_axis).x, origin_c.y - axis_length * Tac(x_axis).y, origin_c.x, origin_c.y, "red");
    svg.addLine(origin_c.x + axis_length * Tac(y_axis).x, origin_c.y - axis_length * Tac(y_axis).y, origin_c.x, origin_c.y, "green");
    svg.addText(origin_c.x, origin_c.y, "{c}");

    Point2D pa{}, pb{}, pc{};
    std::cout << "Enter point p_a:" << std::endl;
    std::cin >> pa;

    pb = Tba(pa);
    pc = Tca(pa);

    std::cout << "p_a " << pa << "\n";
    std::cout << "p_b " << pb << "\n";
    std::cout << "p_c " << pc << std::endl;

    // Draw p_a
    svg.addCircle( origin_a.x + unit_length * pa.x, origin_a.y - unit_length * pa.y, 5, "purple");

    // Draw p_b
    svg.addCircle( origin_a.x + unit_length * Tab(pb).x, origin_a.y - unit_length * Tab(pb).y, 5, "brown");

    // Draw p_b
    svg.addCircle( origin_a.x + unit_length * Tac(pc).x, origin_a.y - unit_length * Tac(pc).y, 5, "orange");

    Vector2D vb{}, vb_hat{}, va{}, vc{};

    std::cout << "Enter vector v_b:" << std::endl;
    std::cin >> vb;

    vb_hat = normalizeVector(vb);

    va = Tab(vb);
    vc = Tcb(vb);
    
    std::cout << "v^_b " << vb_hat << "\n";
    std::cout << "v_a " << va << "\n";
    std::cout << "v_b " << vb << "\n";
    std::cout << "v_c " << vc << std::endl;
    
    // Draw v^_b
    svg.addLine(origin_b.x + axis_length * Tab(vb_hat).x, origin_b.y - axis_length * Tab(vb_hat).y, origin_b.x, origin_b.y, "brown");

    // Draw v_a
    svg.addLine(origin_a.x + axis_length * va.x, origin_a.y - axis_length * va.y, origin_a.x, origin_a.y, "purple");

    // Draw v_c
    svg.addLine(origin_c.x + axis_length * Tac(vc).x, origin_c.y - axis_length * Tac(vc).y, origin_c.x, origin_c.y, "orange");

    // Save the provided SVG code to the file
    svg.saveToFile("tmp/frames.svg", svgCode);

    Twist2D Va, Vb, Vc;

    std::cout << "Enter twist V_b:" << std::endl;
    std::cin >> Vb;
    Va = Tab(Vb);
    Vc = Tcb(Vb);
    std::cout << "V_a " << Va << "\n";
    std::cout << "V_b " << Vb << "\n";
    std::cout << "V_c " << Vc << std::endl;

    return 0;
}