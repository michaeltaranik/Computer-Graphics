//---------------------------
// definition of the plane
//---------------------------
var plane_vertices = [
];
var plane_normals = [
];
var plane_colors = [
];


//---------------------------
// definition of the cube
//---------------------------
var cube_vertices = [
];
var cube_colors = [
];
var cube_normals = [
];


//---------------------------
// definition of the sphere
//---------------------------
var sphere_vertices = [];
var sphere_colors = [];
function create_sphere(){
    let step = 0.01;
    for(let u = 0; u < 1; u = u + step){
        for(let v = 0; v < 1; v = v + step){
            let t = Math.sin(Math.PI*v);

            let x1 = t*Math.cos(2*Math.PI*u);
            let z1 = t*Math.sin(2*Math.PI*u);
            let y1 = Math.cos(Math.PI*v);

            let x4 = t*Math.cos(2*Math.PI*(u+step));
            let z4 = t*Math.sin(2*Math.PI*(u+step));
            let y4 = Math.cos(Math.PI*v);



            t = Math.sin(Math.PI*(v+step));
            let x2 = t*Math.cos(2*Math.PI*u);
            let z2 = t*Math.sin(2*Math.PI*u);
            let y2 = Math.cos(Math.PI*(v+step));

            let x3 = t*Math.cos(2*Math.PI*(u+step));
            let z3 = t*Math.sin(2*Math.PI*(u+step));
            let y3 = Math.cos(Math.PI*(v+step));

            sphere_vertices.push(x1,y1,z1,x3,y3,z3,x2,y2,z2);
            sphere_vertices.push(x1,y1,z1,x4,y4,z4,x3,y3,z3);

            for(let k = 0; k < 6; k++){
                sphere_colors.push(1,1,1);
            }

        }
    }
    //making the sphere a unit sphere
    for(let i = 0; i < sphere_vertices.length; i++){
        sphere_vertices[i] = sphere_vertices[i]/2;
    }
}

create_sphere();
