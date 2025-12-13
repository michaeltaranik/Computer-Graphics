// ====== GROUND PLANE ======
// two triangles forming a 1x1 square centered at origin on xz-plane
var plane_vertices = [
    // triangle 1
    -0.5, 0.0, -0.5,  // vertex 0
     0.5, 0.0, -0.5,  // vertex 1
    -0.5, 0.0,  0.5,  // vertex 2
    
    // triangle 2
    -0.5, 0.0,  0.5,  // vertex 2
     0.5, 0.0, -0.5,  // vertex 1
     0.5, 0.0,  0.5   // vertex 3
];

// normals for plane (all pointing up in y+ direction)
var plane_normals = [
    0.0, 1.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 1.0, 0.0
];

// colors for plane (brownish/taupe)
var plane_colors = [];
for (let i = 0; i < 6; i++) {
    plane_colors.push(0.6, 0.5, 0.4); // brownish/taupe color
}

// ====== cube ======
// unit cube centered at origin (12 triangles = 36 vertices)
var cube_vertices = [];
var cube_normals = [];
var cube_colors = [];

// helper function to add a cube face
function addCubeFace(vertices, normals, colors, v1, v2, v3, v4, normal, color) {
    // Triangle 1
    vertices.push(...v1, ...v2, ...v3);
    normals.push(...normal, ...normal, ...normal);
    colors.push(...color, ...color, ...color);
    
    // Triangle 2  
    vertices.push(...v1, ...v3, ...v4);
    normals.push(...normal, ...normal, ...normal);
    colors.push(...color, ...color, ...color);
}

// Define cube vertices
// front face (red)
addCubeFace(cube_vertices, cube_normals, cube_colors,
    [-0.5, -0.5,  0.5], [ 0.5, -0.5,  0.5], [ 0.5,  0.5,  0.5], [-0.5,  0.5,  0.5],
    [0, 0, 1], [1.0, 0.0, 0.0]);

// back face (green)
addCubeFace(cube_vertices, cube_normals, cube_colors,
    [ 0.5, -0.5, -0.5], [-0.5, -0.5, -0.5], [-0.5,  0.5, -0.5], [ 0.5,  0.5, -0.5],
    [0, 0, -1], [0.0, 1.0, 0.0]);

// top face (blue)
addCubeFace(cube_vertices, cube_normals, cube_colors,
    [-0.5,  0.5,  0.5], [ 0.5,  0.5,  0.5], [ 0.5,  0.5, -0.5], [-0.5,  0.5, -0.5],
    [0, 1, 0], [0.0, 0.0, 1.0]);

// bottom face (yellow)
addCubeFace(cube_vertices, cube_normals, cube_colors,
    [-0.5, -0.5, -0.5], [ 0.5, -0.5, -0.5], [ 0.5, -0.5,  0.5], [-0.5, -0.5,  0.5],
    [0, -1, 0], [1.0, 1.0, 0.0]);

// right face (magenta)
addCubeFace(cube_vertices, cube_normals, cube_colors,
    [ 0.5, -0.5,  0.5], [ 0.5, -0.5, -0.5], [ 0.5,  0.5, -0.5], [ 0.5,  0.5,  0.5],
    [1, 0, 0], [1.0, 0.0, 1.0]);

// left face (cyan)
addCubeFace(cube_vertices, cube_normals, cube_colors,
    [-0.5, -0.5, -0.5], [-0.5, -0.5,  0.5], [-0.5,  0.5,  0.5], [-0.5,  0.5, -0.5],
    [-1, 0, 0], [0.0, 1.0, 1.0]);


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
                sphere_colors.push(1,0,1);
            }

        }
    }
    //making the sphere a unit sphere
    for(let i = 0; i < sphere_vertices.length; i++){
        sphere_vertices[i] = sphere_vertices[i]/2;
    }
}

create_sphere();

var sphere_normals = [];
function createSphereNormals() {
    sphere_normals = [...sphere_vertices];
}
createSphereNormals();
