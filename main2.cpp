#include <raylib.h>
#include <raymath.h>   // Include for vector math functions
#include <rlgl.h>      // Include for raylib OpenGL functionality


#include <math.h>      // For mathematical functions like sinf(), cosf()
#include <vector>
#include <string>
// Define the Character structure
typedef struct Character {
    Vector3 position;
    float speed;
} Character;

typedef struct {
    Camera camera;
    float yaw;
    float pitch;
} FreeFlyCamera;

// Function to update the character's position (if needed)
void UpdateCharacter(Character *character) 
{
    character->position.z += character->speed;
}

// Function to create box vertices
std::vector<Vector3> createBox(Vector3 pos, float width, float height, float depth)
{
    std::vector<Vector3> vertices;

    // Calculate the positions of the 8 corners of the box
    Vector3 v[8];
    v[0] = (Vector3){ pos.x - width / 2, pos.y + height / 2, pos.z + depth / 2 }; // Left Top Front
    v[1] = (Vector3){ pos.x + width / 2, pos.y + height / 2, pos.z + depth / 2 }; // Right Top Front
    v[2] = (Vector3){ pos.x + width / 2, pos.y + height / 2, pos.z - depth / 2 }; // Right Top Back
    v[3] = (Vector3){ pos.x - width / 2, pos.y + height / 2, pos.z - depth / 2 }; // Left Top Back
    v[4] = (Vector3){ pos.x - width / 2, pos.y - height / 2, pos.z + depth / 2 }; // Left Bottom Front
    v[5] = (Vector3){ pos.x + width / 2, pos.y - height / 2, pos.z + depth / 2 }; // Right Bottom Front
    v[6] = (Vector3){ pos.x + width / 2, pos.y - height / 2, pos.z - depth / 2 }; // Right Bottom Back
    v[7] = (Vector3){ pos.x - width / 2, pos.y - height / 2, pos.z - depth / 2 }; // Left Bottom Back

    // Define the 12 triangles composing the box
    // Each face has 2 triangles

    // Front face (+Z)
    vertices.push_back(v[0]); vertices.push_back(v[4]); vertices.push_back(v[5]);
    vertices.push_back(v[0]); vertices.push_back(v[5]); vertices.push_back(v[1]);

    // Right face (+X)
    vertices.push_back(v[1]); vertices.push_back(v[5]); vertices.push_back(v[6]);
    vertices.push_back(v[1]); vertices.push_back(v[6]); vertices.push_back(v[2]);

    // Back face (-Z)
    vertices.push_back(v[2]); vertices.push_back(v[6]); vertices.push_back(v[7]);
    vertices.push_back(v[2]); vertices.push_back(v[7]); vertices.push_back(v[3]);

    // Left face (-X)
    vertices.push_back(v[3]); vertices.push_back(v[7]); vertices.push_back(v[4]);
    vertices.push_back(v[3]); vertices.push_back(v[4]); vertices.push_back(v[0]);

    // Top face (+Y)
    vertices.push_back(v[0]); vertices.push_back(v[1]); vertices.push_back(v[2]);
    vertices.push_back(v[0]); vertices.push_back(v[2]); vertices.push_back(v[3]);

    // Bottom face (-Y)
    vertices.push_back(v[4]); vertices.push_back(v[7]); vertices.push_back(v[6]);
    vertices.push_back(v[4]); vertices.push_back(v[6]); vertices.push_back(v[5]);

    // Now, the 'vertices' vector contains 36 vertices (12 triangles * 3 vertices)

    return vertices;
}

void GrassWindSway(Vector3& vertices1, Vector3& vertices2, Vector3& vertices3, Vector3 windPosition, float windForce)
{

}


void RotateTriangle(Vector3& vertices1, Vector3& vertices2, Vector3& vertices3, float speed)
{
    // Calculate the centroid of the triangle
    Vector3 centroid = {
        (vertices1.x + vertices2.x + vertices3.x) / 3,
        (vertices1.y + vertices2.y + vertices3.y) / 3,
        (vertices1.z + vertices2.z + vertices3.z) / 3
    };
    // Translate vertices to origin (centroid)
    Vector3 v0 = Vector3Subtract(vertices1, centroid);
    Vector3 v1 = Vector3Subtract(vertices2, centroid);
    Vector3 v2 = Vector3Subtract(vertices3, centroid);

    // Rotate each vertex around the y-axis
    v0 = Vector3RotateByAxisAngle(v0, {0, 1, 0}, speed);
    v1 = Vector3RotateByAxisAngle(v1, {0, 1, 0}, speed);
    v2 = Vector3RotateByAxisAngle(v2, {0, 1, 0}, speed);

    // Translate vertices back to their original position
    vertices1 = Vector3Add(v0, centroid);
    vertices2 = Vector3Add(v1, centroid);
    vertices3 = Vector3Add(v2, centroid);
}

std::vector<Vector3> createRandomTriangles(int numShapes, float minwidth, float maxwidth, float minheight, float maxheight, float minX, float maxX, float minY, float maxY, float minZ, float maxZ)
{
    std::vector<Vector3> vertices;
    for (int i = 0; i < numShapes; i++)
    {
        float randX = GetRandomValue((int)(minX*100), (int)(maxX*100)) / 100.0f;
        float randY = GetRandomValue((int)(minY*100), (int)(maxY*100)) / 100.0f;
        float randZ = GetRandomValue((int)(minZ*100), (int)(maxZ*100)) / 100.0f;
        float width = GetRandomValue((int)(minwidth*100), (int)(maxwidth*100)) / 100.0f;
        float height = GetRandomValue((int)(minheight*100), (int)(maxheight*100)) / 100.0f;

        Vector3 position = { randX, randY, randZ };

        Vector3 baseA = { position.x - width/2, position.y, position.z - width/2 };
        Vector3 baseB = { position.x + width/2, position.y, position.z - width/2 };
        Vector3 baseC = { position.x,          position.y, position.z + width/2 };
        Vector3 apex  = { position.x, position.y + height, position.z };

        // Face 1
        vertices.push_back(apex);
        vertices.push_back(baseB);
        vertices.push_back(baseA);

        // Face 2
        vertices.push_back(apex);
        vertices.push_back(baseA);
        vertices.push_back(baseC);

        // Face 3
        vertices.push_back(apex);
        vertices.push_back(baseC);
        vertices.push_back(baseB);

        // Base face
        vertices.push_back(baseA);
        vertices.push_back(baseB);
        vertices.push_back(baseC);
    }
    return vertices;
}
std::vector<Vector3> createLineAroundTriangles(std::vector<Vector3> vertices)
{
    std::vector<Vector3> linevertices;
    for (std::vector<Vector3>::size_type i = 0; i < vertices.size(); i+=3)
    {
        Vector3 v1 = vertices[i];
        Vector3 v2 = vertices[i+1];
        Vector3 v3 = vertices[i+2];

        // Line 1
        linevertices.push_back(v1);
        linevertices.push_back(v2);

        // Line 2
        linevertices.push_back(v2);
        linevertices.push_back(v3);

        // Line 3
        linevertices.push_back(v3);
        linevertices.push_back(v1);
    }
    return linevertices;
}

void moveCamera(FreeFlyCamera *ffc, double rotSpeed, double moveSpeed) 
{
    // Mouse movement
    Vector2 mouseDelta = GetMouseDelta();
    ffc->yaw -= mouseDelta.x * rotSpeed;
    ffc->pitch -= mouseDelta.y * rotSpeed;

    // Clamp pitch to avoid gimbal lock
    if (ffc->pitch > PI / 2.0f) ffc->pitch = PI / 2.0f;
    if (ffc->pitch < -PI / 2.0f) ffc->pitch = -PI / 2.0f;

    // Calculate forward and right vectors
    Vector3 forward = {
        cosf(ffc->pitch) * sinf(ffc->yaw),
        sinf(ffc->pitch),
        cosf(ffc->pitch) * cosf(ffc->yaw)
    };
    Vector3 right = {
        cosf(ffc->yaw),
        0.0f,
        -sinf(ffc->yaw)
    };

    // Movement input
    Vector3 movement = {0};
    if (IsKeyDown(KEY_W)) movement = Vector3Add(movement, forward);
    if (IsKeyDown(KEY_S)) movement = Vector3Subtract(movement, forward);
    if (IsKeyDown(KEY_A)) movement = Vector3Add(movement, right);
    if (IsKeyDown(KEY_D)) movement = Vector3Subtract(movement, right);
    if (IsKeyDown(KEY_LEFT_SHIFT)) movement = Vector3Add(movement, (Vector3){0, 1, 0});
    if (IsKeyDown(KEY_LEFT_CONTROL)) movement = Vector3Subtract(movement, (Vector3){0, 1, 0});

    // Normalize movement vector to prevent faster diagonal movement
    if (Vector3Length(movement) > 0.0f) movement = Vector3Normalize(movement);

    // Update camera position and target
    ffc->camera.position = Vector3Add(ffc->camera.position, Vector3Scale(movement, moveSpeed));
    ffc->camera.target = Vector3Add(ffc->camera.position, forward);
}

int main()
{
    // Initialization
    const int screenWidth = 900;
    const int screenHeight = 650;
    
    InitWindow(screenWidth, screenHeight, "3D Triangle at Character Location");
    SetTargetFPS(9999); // Set our game to run at 60 frames-per-second

    //rlEnableSmoothLines(); // Enable line smoothing for a smoother display
    // Hide the cursor and start capturing it
    DisableCursor();
    SetMousePosition(screenWidth / 2, screenHeight / 2); // Center the mouse position




    // Define the camera to look into our 3D world
    Camera camera = { 0 };
    camera.position = (Vector3){ 0.0f, 5.0f, -20.0f };  // Camera position
    camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };    // Camera looking at point
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };        // Camera up vector (rotation towards target)
    camera.fovy = 45.0f;                              // Camera field-of-view Y
    camera.projection = CAMERA_PERSPECTIVE;           // Camera mode type

    
    FreeFlyCamera ffc = { camera, 0.0f, 0.0f };

    // Initialize the character
    Character character = { 0 };
    character.position = (Vector3){ 0.0f, 0.0f, 0.0f };
    character.speed = 0.1f;

    std::vector<Vector3> vertices;
    vertices = createRandomTriangles(10000, 1.5f, 1.7f, 1.0f, 7.0f, -300, 300, 0, 0, -300, 300);
    Mesh grassmesh = {0};
    grassmesh.triangleCount = vertices.size() / 3;
    grassmesh.vertexCount = vertices.size();
    grassmesh.vertices = (float *)malloc(vertices.size() * 3 * sizeof(float));
    Material material = LoadMaterialDefault();
    material.maps[MATERIAL_MAP_DIFFUSE].color = GREEN;
    for (size_t i = 0; i < vertices.size(); i++) 
    {
        grassmesh.vertices[i * 3] = vertices[i].x;
        grassmesh.vertices[i * 3 + 1] = vertices[i].y;
        grassmesh.vertices[i * 3 + 2] = vertices[i].z;
    }
    UploadMesh(&grassmesh, false);



    // Main game loop
    //float trianglerRotationSpeed = 1.0f;
    float windForce = 0.01f;

    Shader lineShader = LoadShader("shaders/lineshader.vx", "shaders/lineshader.fs");
    
    Vector4 lineColor = {0.0f, 0.0f, 0.0f, 1.0f};  


    int locModel = GetShaderLocation(lineShader, "model");
    int locView = GetShaderLocation(lineShader, "view");
    int locProjection = GetShaderLocation(lineShader, "projection");

    float edgeThreshold = 0.06f; // Adjust as needed for edge thickness
    SetShaderValue(lineShader, GetShaderLocation(lineShader, "edgeThreshold"), &edgeThreshold, SHADER_UNIFORM_FLOAT);
    

    int locLineColor = GetShaderLocation(lineShader, "lineColor");
    SetShaderValue(lineShader, locLineColor, &lineColor, SHADER_UNIFORM_VEC4);

    
    while (!WindowShouldClose()) // Detect window close button or ESC key
    {

        double deltaTime = GetFrameTime();


        moveCamera(&ffc, 0.002, 100.0*deltaTime);



        SetShaderValueMatrix(lineShader, locModel, MatrixIdentity());
        SetShaderValueMatrix(lineShader, locView, GetCameraMatrix(ffc.camera)); // Will be updated each frame
        SetShaderValueMatrix(lineShader, locProjection, MatrixPerspective(ffc.camera.fovy * DEG2RAD, (float)screenWidth / (float)screenHeight, 0.01f, 10000.0f));

        // Start Drawing
        BeginDrawing();
            ClearBackground(BLUE);

            BeginMode3D(ffc.camera);
            BeginShaderMode(lineShader);

            Material shadermat = LoadMaterialDefault();
            shadermat.shader = lineShader;

            for (std::vector<Vector3>::size_type i = 0; i < vertices.size(); i+=3)
            {
                Vector3 triangle[3] = { vertices[i], vertices[i + 1], vertices[i + 2] };
                SetShaderValueV(lineShader, GetShaderLocation(lineShader, "vertices"), triangle, SHADER_UNIFORM_VEC3, 3);
                DrawMesh(grassmesh, shadermat, MatrixIdentity());

            }
            
            EndShaderMode();
            DrawMesh(grassmesh, material, MatrixIdentity());

            

            DrawGrid(20, 1.0f);

            EndMode3D();

            DrawText("Use WASD keys to move the camera", 10, 10, 20, DARKGRAY);
            DrawFPS(screenWidth - 90, 10);
            std::string cameraPos = "Camera Position: " + std::to_string(camera.position.x) + ", " + std::to_string(camera.position.y) + ", " + std::to_string(camera.position.z);
            DrawText(cameraPos.c_str(), 10, 30, 20, DARKGRAY);
            std::string cameraTarget = "Camera Target: " + std::to_string(camera.target.x) + ", " + std::to_string(camera.target.y) + ", " + std::to_string(camera.target.z);
            DrawText(cameraTarget.c_str(), 10, 50, 20, DARKGRAY);
            std::string windForcestring = "Wind Force: " + std::to_string(windForce);
            DrawText(windForcestring.c_str(), 10, 70, 20, DARKGRAY);
            std::string verticesAmount = "Vertices: " + std::to_string(vertices.size());
            DrawText(verticesAmount.c_str(), 10, 90, 20, DARKGRAY);
            

        EndDrawing();
        // End Drawing
    }

    // De-Initialization
    UnloadMesh(grassmesh); // Unload mesh data
    UnloadMaterial(material); // Unload material data

    CloseWindow();        // Close window and OpenGL context

    return 0;
