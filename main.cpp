
#include <raylib.h>
#include <raymath.h>   // Include for vector math functions
#include <rlgl.h>      // Include for raylib OpenGL functionality

#include <iostream>    // For cout and cin
#include <math.h>      // For mathematical functions like sinf(), cosf()
#include <vector>
#include <string>
#include <array>
#include <unordered_set>
#include <unistd.h>
#include <algorithm>


#define WORLD_SIZE 500
#define CELL_SIZE 20

// Define the Character structure
typedef struct Character {
    int id;
    Vector3 position;
    float width;
    float height;
    float length;
    float speed;
    bool target;
    Vector3 targetPosition;
    std::array<int, 4> cells = {0, 0, 0, 0};
    std::vector<Vector3> vertices;
    Vector3 startposition;

    Character() : position{0.0f, 0.0f, 0.0f}, width(1), height(1), length(1), speed(1.0f), target(false), targetPosition{0.0f, 0.0f, 0.0f} {}
} Character;

typedef struct {
    Camera camera;
    float yaw;
    float pitch;
} FreeFlyCamera;

typedef struct threeSidedTriangle {
    Vector3 vertices[12];
    Vector3 centerPosition;
    float health = 1;
    int id;

} threeSidedTriangle;

struct VertexData {
    Vector3 position; // World-space position
    Color color;      // Vertex color
};

typedef struct {
    int x;
    int y;
    int z;
    int width;
    int depth;
    std::vector<Character*> characters;
    std::vector<threeSidedTriangle*> triangles;
} CollisionCell;


void InitCollisionCellObjects(std::array<CollisionCell, WORLD_SIZE*WORLD_SIZE/CELL_SIZE/CELL_SIZE> &collisionCells, std::vector<Character> &characters, std::vector<threeSidedTriangle> &triangles)
{
    //std::cout << "InitCollisionCellObjects" << std::endl;
    for (CollisionCell &cell : collisionCells)
    {
        cell.triangles.clear();
        cell.characters.clear();
    }
    for (Character &character : characters)
    {
        for (CollisionCell &cell : collisionCells)
        {
            if (character.position.x >= cell.x && character.position.x < cell.x + cell.width &&
                character.position.z >= cell.z && character.position.z < cell.z + cell.depth)
            {
                cell.characters.push_back(&character);
                for (int i = 0; i < 4; i++)
                {
                    character.cells[i] = (int)((character.position.x + WORLD_SIZE / 2) / CELL_SIZE) * (WORLD_SIZE / CELL_SIZE) + (int)((character.position.z + WORLD_SIZE / 2) / CELL_SIZE);
                }
            }
        }
    }
    int inx = 0;
    for (threeSidedTriangle &triangle : triangles)
    {
        triangle.id = inx;
        inx++;
        for (CollisionCell &cell : collisionCells)
        {
            if (triangle.centerPosition.x >= cell.x && triangle.centerPosition.x < cell.x + cell.width &&
                triangle.centerPosition.z >= cell.z && triangle.centerPosition.z < cell.z + cell.depth)
            {
                cell.triangles.push_back(&triangle);
            }
        }
    }
}

// Function to update the character's position (if needed)
void UpdateCharacter(Character &character, std::array<CollisionCell, WORLD_SIZE*WORLD_SIZE/CELL_SIZE/CELL_SIZE> &collisionCells, std::vector<int> &trianglesToUpdate, double deltaTime)
{
    std::array<Vector2, 4> corners = {
        Vector2{character.position.x - character.width / 2, character.position.z - character.length / 2},
        Vector2{character.position.x + character.width / 2, character.position.z - character.length / 2},
        Vector2{character.position.x + character.width / 2, character.position.z + character.length / 2},
        Vector2{character.position.x - character.width / 2, character.position.z + character.length / 2}
    };
   
    for (int i = 0; i < 4; i++)
    {
        
        if (corners[i].x > collisionCells[character.cells[i]].x || corners[i].x > collisionCells[character.cells[i]].x + collisionCells[character.cells[i]].width ||
            corners[i].y > collisionCells[character.cells[i]].z || corners[i].y > collisionCells[character.cells[i]].z + collisionCells[character.cells[i]].depth)
        {
            int inx = 0;
            for (auto cell : collisionCells[character.cells[i]].characters)
            {
                if (cell->id == character.id)
                {
                    collisionCells[character.cells[i]].characters.erase(collisionCells[character.cells[i]].characters.begin()+inx);
                    int newCellIndex = (int)((character.position.x + WORLD_SIZE / 2) / CELL_SIZE) * (WORLD_SIZE / CELL_SIZE) + (int)((character.position.z + WORLD_SIZE / 2) / CELL_SIZE);
                    CollisionCell &newCell = collisionCells[newCellIndex];
                    newCell.characters.push_back(&character);
                    character.cells[i] = newCellIndex;
                    break;
                }
                inx++;
            }
        }
    }

    bool triangleFound = false;
    for (int i = 0; i < 4; i++)
    {
        for (auto &triangle : collisionCells[character.cells[i]].triangles)
        {
            if (Vector3Distance(character.position, triangle->centerPosition) < 5.0f)
            {
                character.target = false;
                triangle->health -= 1000;
                trianglesToUpdate.push_back(triangle->id);
    
                triangleFound = true;

            }
        }
        if (triangleFound)
        {
            break;
        }
    }
    if (Vector3Distance(character.position, character.targetPosition) < 1.0f)
    {
        character.target = false;
    }

    if (character.target == false)
    {
        for (int charCell : character.cells)
        {
            if (collisionCells[charCell].triangles.size() > 0)
            {
                float nearestDistance = WORLD_SIZE;
                for (auto triangle : collisionCells[charCell].triangles)
                {
                    if (Vector3Distance(character.position, triangle->centerPosition) < nearestDistance)
                    {
                        nearestDistance = Vector3Distance(character.position, triangle->centerPosition);
                        //std::cout << "nearest distance: " << nearestDistance << std::endl;
                        character.target = true;
                        character.targetPosition = triangle->centerPosition;
                    }
                }
            }
            if(character.cells[0] == character.cells[2])
            {   
                if (character.cells[0] == character.cells[1] && character.cells[0] == character.cells[3])
                {
                    break;
                }
                else
                {
                    std::cout << "something is wrong... " << character.cells[0] << "  |  " << character.cells[1] << "  |  " << character.cells[2] << "  |  " << character.cells[3] << std::endl;
                }
            }
        }
        if (character.target == false)
        {
            float nearestDistance = WORLD_SIZE;
            for (auto cell : collisionCells)
            {
                for (auto triangle : cell.triangles)
                {
                    if (Vector3Distance(character.position, triangle->centerPosition) < nearestDistance)
                    {
                        nearestDistance = Vector3Distance(character.position, triangle->centerPosition);
                        character.target = true;
                        character.targetPosition = triangle->centerPosition;
                    }
                }
            }
        }
    }
    Vector3 direction;
    
    if (character.target)
    {
        direction = Vector3Normalize(Vector3Subtract(character.targetPosition, character.position));
    }
    else
    {
        direction = Vector3Normalize(Vector3Subtract({character.startposition}, character.position));
    }
    if (Vector3Distance(character.position, character.targetPosition) < character.speed * deltaTime)
    {
        character.position = character.targetPosition;
    }
    else
    {
        character.position = Vector3Add(character.position, Vector3Scale(direction, character.speed * deltaTime));
    }

}

// Function to create box vertices
std::vector<Vector3> createBox(Vector3 pos, float width, float height, float depth)
{
    std::vector<Vector3> vertices;

    // Calculate the positions of the 8 corners of the box
    Vector3 v[8];
    v[0] = (Vector3){ pos.x - width / 2, pos.y + height, pos.z + depth / 2 }; // Left Top Front
    v[1] = (Vector3){ pos.x + width / 2, pos.y + height, pos.z + depth / 2 }; // Right Top Front
    v[2] = (Vector3){ pos.x + width / 2, pos.y + height, pos.z - depth / 2 }; // Right Top Back
    v[3] = (Vector3){ pos.x - width / 2, pos.y + height, pos.z - depth / 2 }; // Left Top Back
    v[4] = (Vector3){ pos.x - width / 2, pos.y, pos.z + depth / 2 }; // Left Bottom Front
    v[5] = (Vector3){ pos.x + width / 2, pos.y, pos.z + depth / 2 }; // Right Bottom Front
    v[6] = (Vector3){ pos.x + width / 2, pos.y, pos.z - depth / 2 }; // Right Bottom Back
    v[7] = (Vector3){ pos.x - width / 2, pos.y, pos.z - depth / 2 }; // Left Bottom Back

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

//void GrassWindSway(Vector3& vertices1, Vector3& vertices2, Vector3& vertices3, Vector3 windPosition, float windForce)
//{
//
//}


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

std::vector<threeSidedTriangle> createRandomTriangles(int numShapes, float minwidth, float maxwidth, float minheight, float maxheight, float minX, float maxX, float minY, float maxY, float minZ, float maxZ)
{
    std::vector<threeSidedTriangle> triangles;
    for (int i = 0; i < numShapes; i++)
    {
        triangles.push_back(threeSidedTriangle());
        float randX = GetRandomValue((int)(minX*100), (int)(maxX*100)) / 100.0f;
        float randY = GetRandomValue((int)(minY*100), (int)(maxY*100)) / 100.0f;
        float randZ = GetRandomValue((int)(minZ*100), (int)(maxZ*100)) / 100.0f;
        float width = GetRandomValue((int)(minwidth*100), (int)(maxwidth*100)) / 100.0f;
        float height = GetRandomValue((int)(minheight*100), (int)(maxheight*100)) / 100.0f;

        Vector3 position = { randX, randY, randZ };
        triangles[i].centerPosition = position;

        Vector3 baseA = { position.x - width/2, position.y, position.z - width/2 };
        Vector3 baseB = { position.x + width/2, position.y, position.z - width/2 };
        Vector3 baseC = { position.x,          position.y, position.z + width/2 };
        Vector3 apex  = { position.x, position.y + height, position.z };

        // Face 1
        triangles[i].vertices[0] = apex;
        triangles[i].vertices[1] = baseB;
        triangles[i].vertices[2] = baseA;

        // Face 2
        triangles[i].vertices[3] = apex;
        triangles[i].vertices[4] = baseA;
        triangles[i].vertices[5] = baseC;

        // Face 3
        triangles[i].vertices[6] = apex;
        triangles[i].vertices[7] = baseC;
        triangles[i].vertices[8] = baseB;

        // Base face
        triangles[i].vertices[9] = baseA;
        triangles[i].vertices[10] = baseB;
        triangles[i].vertices[11] = baseC;
    }
    return triangles;
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

void flyCamera(FreeFlyCamera *ffc, double rotSpeed, double moveSpeed)
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
void walkCamera(FreeFlyCamera *ffc, double rotSpeed, double moveSpeed, bool &isGrounded, float jumpSpeed, float gravity) 
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
        0.0f, // No vertical movement for walking
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

    // Normalize movement vector to prevent faster diagonal movement
    if (Vector3Length(movement) > 0.0f) movement = Vector3Normalize(movement);

    // Apply movement
    ffc->camera.position = Vector3Add(ffc->camera.position, Vector3Scale(movement, moveSpeed));

    // Jumping and gravity
    if (isGrounded && IsKeyPressed(KEY_SPACE)) {
        ffc->camera.position.y += jumpSpeed;
        isGrounded = false;
    }

    if (!isGrounded) {
        ffc->camera.position.y -= gravity * GetFrameTime();
    }

    // Check if grounded (simple ground check, can be improved with collision detection)
    if (ffc->camera.position.y <= 0.0f) {
        ffc->camera.position.y = 0.0f;
        isGrounded = true;
    }

    // Update camera target
    ffc->camera.target = Vector3Add(ffc->camera.position, forward);
}

int main()
{
    // Initialization
    const int screenWidth = 1200;
    const int screenHeight = 900;

    //If shader folder is not present, copy it from ../resources
    std::string shaderPath = std::string(GetWorkingDirectory()) + "\\resources";
    if (!DirectoryExists(shaderPath.c_str()))
    {
        std::string resourcesPath = std::string(GetWorkingDirectory()).substr(0, std::string(GetWorkingDirectory()).find_last_of("\\")) + "\\resources";

        std::string shaderResourcesPath = resourcesPath + "\\shaders";
        std::string targetPath = std::string(GetWorkingDirectory()) + "\\resources";
        #ifdef _WIN32
        std::string copyCommand = "xcopy /E /I \"" + resourcesPath + "\" \"" + targetPath + "\"";
        #else
        std::string copyCommand = "cp -r \"" + resourcesPath + "\" \"" + targetPath + "\"";
        #endif
        system(copyCommand.c_str());
        std::cout << "\ntries to copy from " << resourcesPath << " to " << targetPath << "\n\n";
    }
    
    InitWindow(screenWidth, screenHeight, "3D Triangle at Character Location");
    //move window to another screen
    SetWindowMonitor(1);
    SetTargetFPS(9999); 

    //rlEnableSmoothLines(); // Enable line smoothing for a smoother display
    // Hide the cursor and start capturing it
    DisableCursor();
    SetMousePosition(screenWidth / 2, screenHeight / 2); // Center the mouse position

    std::array<CollisionCell, WORLD_SIZE*WORLD_SIZE/CELL_SIZE/CELL_SIZE> collisionCells;
    int cellIndex = 0;
    for (int x = -WORLD_SIZE/2; x < WORLD_SIZE/2; x += CELL_SIZE)
    {
        for (int z = -WORLD_SIZE/2; z < WORLD_SIZE/2; z += CELL_SIZE)
        {
            collisionCells[cellIndex].x = x;
            collisionCells[cellIndex].y = 0;
            collisionCells[cellIndex].z = z;
            collisionCells[cellIndex].width = CELL_SIZE;
            collisionCells[cellIndex].depth = CELL_SIZE;
            cellIndex++;
        }
    }


    // Define the camera to look into our 3D world
    Camera camera = { 0 };
    camera.position = (Vector3){ 0.0f, 5.0f, -20.0f };  // Camera position
    camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };    // Camera looking at point
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };        // Camera up vector (rotation towards target)
    camera.fovy = 45.0f;                              // Camera field-of-view Y
    camera.projection = CAMERA_PERSPECTIVE;           // Camera mode type
    std::cout << "cull distance: " << rlGetCullDistanceFar() << "\n"; // Get the far cull distance

    FreeFlyCamera ffc = { camera, 0.0f, 0.0f };

    float cameraYaw = 0.0f;
    float cameraPitch = 0.0f;
    // Create Characters
    std::vector<Character> characters;
    for (int i = 0; i < 0; i++)
    {
        characters.push_back(Character());
        characters[i].id = i;
        characters[i].position = {(float)GetRandomValue(-WORLD_SIZE/2, WORLD_SIZE/2), 0.0f, (float)GetRandomValue(-WORLD_SIZE/2, WORLD_SIZE/2)};
        characters[i].speed = 50;//(float)GetRandomValue(1, 100);
        characters[i].target = false;
        characters[i].targetPosition = {0.0f, 0.0f, 0.0f};
        characters[i].vertices = createBox(characters[i].position, 1.0f, 3.0f, 1.0f);
        characters[i].cells = {0, 0, 0, 0};
        characters[i].startposition = characters[i].position;
    }

  


    const float cameraSpeed = 40.1f;
    std::vector<threeSidedTriangle> triangles;
    triangles = createRandomTriangles(5000000, 2.5f, 4.0f, 2.0f, 6.0f, -WORLD_SIZE/2, WORLD_SIZE/2, 0, 0, -WORLD_SIZE/2, WORLD_SIZE/2);
    std::vector<VertexData> vertices;
    for (const auto& triangle : triangles) 
    {
        for (int i = 0; i < 12; i++)
        {
            vertices.push_back({triangle.vertices[i], GREEN});
        }
    }
    

    unsigned int vbo, vao;


    vbo = rlLoadVertexBuffer(vertices.data(), vertices.size() * sizeof(VertexData), RL_DYNAMIC_COPY);
    vao = rlLoadVertexArray();

    rlEnableVertexArray(vao);
    rlSetVertexAttribute(0, 3, RL_FLOAT, false, sizeof(VertexData), (int)offsetof(VertexData, position));
    rlEnableVertexAttribute(0);
    rlSetVertexAttribute(1, 4, RL_UNSIGNED_BYTE, true, sizeof(VertexData), (int)offsetof(VertexData, color));
    rlEnableVertexAttribute(1);
    rlDisableVertexArray();

    // Load shader (adjust paths as needed)






    // Main game loop
    //float trianglerRotationSpeed = 1.0f;
    float windForce = 0.01f;

    Shader lineShader = LoadShader("resources/shaders/lineshader.vx", "resources/shaders/lineshader.fs");
    
    //Vector4 lineColor = {0.0f, 0.0f, 0.0f, 1.0f};  // Red lines


    //int locModel = GetShaderLocation(lineShader, "model");
    //int locView = GetShaderLocation(lineShader, "view");
    //int locProjection = GetShaderLocation(lineShader, "projection");

    //float edgeThreshold = 0.1f; // Adjust as needed for edge thickness
    //SetShaderValue(lineShader, GetShaderLocation(lineShader, "edgeThreshold"), &edgeThreshold, SHADER_UNIFORM_FLOAT);
    

    //int locLineColor = GetShaderLocation(lineShader, "lineColor");
    //SetShaderValue(lineShader, locLineColor, &lineColor, SHADER_UNIFORM_VEC4);

    
    std::cout << "Working DIR:::::" << GetWorkingDirectory() << std::endl;
    //material.shader = lineShader;  // Applying your shader to material

    for (CollisionCell &cell : collisionCells)
    {
        cell.triangles.clear();
    }
    for (Character &character : characters)
    {
        character.cells = {0, 0, 0, 0};
        character.target = false;
    }
    triangles.clear();
    vertices.clear();

    for (auto cell : collisionCells)
    {
        cell.characters.reserve(100);
        cell.triangles.reserve(1000000);
    }
    InitCollisionCellObjects(collisionCells, characters, triangles);
    bool needInitCol = false;
    double targetTickRate = 50;
    double tickTime = 1/targetTickRate;
    double cumulativeTime = 0;
    double amountOfShaderTriangles = 0;
    std::cout << "Tick time: " << tickTime << std::endl;
    bool bMinimap = false;
    while (!WindowShouldClose()) // Detect window close button or ESC key
    {

        double deltaTime = GetFrameTime();
        cumulativeTime += deltaTime;
        // Camera movement controls
        // Mouse movement sensitivity

        flyCamera(&ffc, 0.002, 100.0*deltaTime);
        
        //SetShaderValueMatrix(lineShader, locModel, MatrixIdentity());
        //SetShaderValueMatrix(lineShader, locView, GetCameraMatrix(ffc.camera)); // Will be updated each frame
        //SetShaderValueMatrix(lineShader, locProjection, MatrixPerspective(ffc.camera.fovy * DEG2RAD, (float)screenWidth / (float)screenHeight, 1.01f, 10000.0f));
        std::vector<int> trianglesToUpdate;
        bool shaderrefresh = false;
        while (cumulativeTime >= tickTime)
        {
            for (Character &character : characters)
            {
                UpdateCharacter(character, collisionCells, trianglesToUpdate, tickTime);
                character.vertices = createBox(character.position, character.width, character.height, character.length);
            }
            //sort vector by largest id first:
            std::sort(trianglesToUpdate.begin(), trianglesToUpdate.end(), std::greater<int>());

            for (int id : trianglesToUpdate)
            {
                if (triangles[id].health <= 0)
                {
                    shaderrefresh = true;
                    triangles.erase(triangles.begin() + id);
                }
            }

            if (shaderrefresh)
            {
                shaderrefresh = false;
                vertices.clear();
                trianglesToUpdate.clear();
                std::vector<VertexData> fresh;
                for (auto &t : triangles) {
                    for (int i = 0; i < 12; i++) {
                        fresh.push_back({ t.vertices[i], GREEN });
                    }
                }
                vertices.insert(vertices.end(), fresh.begin(), fresh.end());
                amountOfShaderTriangles = vertices.size();
                InitCollisionCellObjects(collisionCells, characters, triangles);
                rlUpdateVertexBuffer(vbo, vertices.data(), vertices.size() * sizeof(VertexData), 0);
            }
            if (IsKeyDown(KEY_KP_1))
            {
                needInitCol = true;
                std::vector<threeSidedTriangle> newTriangles = createRandomTriangles(1000, 2.5f, 4.0f, 2.0f, 6.0f, -WORLD_SIZE/2, WORLD_SIZE/2, 0, 0, -WORLD_SIZE/2, WORLD_SIZE/2);
                triangles.insert(triangles.end(), newTriangles.begin(), newTriangles.end());

                std::vector<VertexData> fresh;
                for (auto &t : newTriangles) {
                    for (int i = 0; i < 12; i++) {
                        fresh.push_back({ t.vertices[i], GREEN });
                    }
                }
                
                vertices.reserve(vertices.size() + fresh.size());
                vertices.insert(vertices.end(), fresh.begin(), fresh.end());
                amountOfShaderTriangles = vertices.size();
                rlUpdateVertexBuffer(vbo, vertices.data(), vertices.size() * sizeof(VertexData), 0);

            }
            cumulativeTime -= tickTime;
        }


        //input
        if (IsKeyReleased(KEY_KP_1) && needInitCol)
        {
            needInitCol = false;
            InitCollisionCellObjects(collisionCells, characters, triangles);
        }
        if (IsMouseButtonPressed(MOUSE_RIGHT_BUTTON))
        {
            for (CollisionCell &cell : collisionCells)
            {
                cell.triangles.clear();
            }
            for (Character &character : characters)
            {
                character.cells = {0, 0, 0, 0};
                character.target = false;
            }
            triangles.clear();
            vertices.clear();
            trianglesToUpdate.clear();
            InitCollisionCellObjects(collisionCells, characters, triangles);
        }
        if (IsKeyPressed(KEY_KP_2))
        {
            bMinimap = !bMinimap;
        }

        // Start Drawing
        BeginDrawing();
            ClearBackground({71, 153, 193, 255});

            BeginMode3D(ffc.camera);

            //Material shadermat = LoadMaterialDefault();
            //shadermat.shader = lineShader;

            DrawPlane({0.f, 0.f, 0.f}, {WORLD_SIZE, WORLD_SIZE}, {52, 127, 42, 255});

            for (Character &character : characters)
            {
                DrawCube(character.position, 1.0f, 3.0f, 1.0f, RED);
            }
            BeginShaderMode(lineShader);
                // Get model-view-projection matrix (ensure correct order: Projection * View * Model)
                Matrix matView = rlGetMatrixModelview();
                Matrix matProj = rlGetMatrixProjection();
                Matrix matMVP = MatrixMultiply(matView, matProj); // Order depends on your transformations!
                SetShaderValueMatrix(lineShader, GetShaderLocation(lineShader, "mvp"), matMVP);

                // Bind VAO and draw
                rlEnableVertexArray(vao);
                rlDrawVertexArray(0, vertices.size()); // Use rlDrawVertexArray
                rlDisableVertexArray();
            EndShaderMode();

            
            //SetShaderValueV(lineShader, GetShaderLocation(lineShader, "vertices"), &vertices[i].vertices, SHADER_UNIFORM_VEC3, 12);
            //DrawMesh(grassmesh, material, MatrixIdentity());

            //draw cells
            int savedtriangles = 0;
            int savedcharacters = 0;
            for (CollisionCell &cell : collisionCells)
            {
                DrawCubeWires((Vector3){(float)cell.x + cell.width/2, 0.0f, (float)cell.z + cell.depth/2}, cell.width, 1.0f, cell.depth, BLACK);
                for (auto &triangle : cell.triangles)
                {
                    savedtriangles++;
                }
                for (auto &character : cell.characters)
                {
                    savedcharacters++;
                }
                //render cell number in the cell in 3d

                //cout memory location of all triangles in cell

                //for (auto &triangle : cell.triangles)
                //{
                //    std::cout << "    cell: " << triangle << "  |  ";
                //}
                
            }

            EndMode3D();

            DrawText("Use WASD keys to move the camera", 10, 10, 20, DARKGRAY);
            DrawRectangle(screenWidth - 90, 10, 90, 20, BLACK);
            DrawFPS(screenWidth - 90, 10);
            std::string cameraPos = "Camera Position: " + std::to_string(camera.position.x) + ", " + std::to_string(camera.position.y) + ", " + std::to_string(camera.position.z);
            DrawText(cameraPos.c_str(), 10, 30, 20, DARKGRAY);
            std::string cameraTarget = "Camera Target: " + std::to_string(camera.target.x) + ", " + std::to_string(camera.target.y) + ", " + std::to_string(camera.target.z);
            DrawText(cameraTarget.c_str(), 10, 50, 20, DARKGRAY);
            std::string windForcestring = "Wind Force: " + std::to_string(windForce);
            DrawText(windForcestring.c_str(), 10, 70, 20, DARKGRAY);
            //std::string verticesAmount = "Vertices: " + std::to_string(vertices.size());
            //DrawText(verticesAmount.c_str(), 10, 90, 20, DARKGRAY);
            std::string savedTriangles = "Triangles in cells: " + std::to_string(savedtriangles);
            DrawText(savedTriangles.c_str(), 10, 90, 20, DARKGRAY);
            //triangles in vertices
            std::string trianglesInVertices = "Triangles in vertices: " + std::to_string(triangles.size());
            DrawText(trianglesInVertices.c_str(), 10, 110, 20, DARKGRAY);
            std::string savedCharacters = "Characters in cells: " + std::to_string(savedcharacters);
            DrawText(savedCharacters.c_str(), 10, 130, 20, DARKGRAY);
            std::string shadertrianglesstring = "Triangles in shader: " + std::to_string(amountOfShaderTriangles);
            DrawText(shadertrianglesstring.c_str(), 10, 150, 20, DARKGRAY);

            //draw a minimap of my cell grid on the top right corner of the screen with red dots for characters and green dots for triangles
            if (bMinimap)
            {
                DrawRectangle(screenWidth - 200 + (camera.position.x + WORLD_SIZE / 2) / 5, 100 + (camera.position.z + WORLD_SIZE / 2) / 5, 5, 5, BLUE);
                for (CollisionCell &cell : collisionCells)
                {
                    for (auto &triangle : cell.triangles)
                    {
                        DrawRectangle(screenWidth - 200 + triangle->centerPosition.x / 5, 100 + triangle->centerPosition.z / 5, 1, 1, GREEN);
                    }
                    for (auto &character : cell.characters)
                    {
                        DrawRectangle(screenWidth - 200 + character->position.x / 5, 100 + character->position.z / 5, 5, 5, RED);
                    }
                }
            }

            //std::string characterspeed = "Character Speed: " + std::to_string(character.speed);
            //DrawText(characterspeed.c_str(), 10, 110, 20, DARKGRAY);
            //std::string chararactertarget = "Character Target: " + std::to_string(character.target);
            //DrawText(chararactertarget.c_str(), 10, 130, 20, DARKGRAY);
            

        EndDrawing();
        // End Drawing
    }

    // De-Initialization
    UnloadShader(lineShader);
    rlUnloadVertexBuffer(vbo);
    rlUnloadVertexArray(vao);

    CloseWindow();        // Close window and OpenGL context

    return 0;
}