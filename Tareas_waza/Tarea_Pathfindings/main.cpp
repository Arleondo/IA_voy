#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <vector>

const int GRID_SIZE = 100;

std::vector<float> vertices;

void generateGrid()
{
    float size = 1.8f;               // tamaño visual de la cuadrícula
    float step = size / GRID_SIZE;

    float start = -size / 2.0f;

    for(int i = 0; i <= GRID_SIZE; i++)
    {
        float pos = start + i * step;

        // vertical
        vertices.push_back(pos);
        vertices.push_back(start);

        vertices.push_back(pos);
        vertices.push_back(start + size);

        // horizontal
        vertices.push_back(start);
        vertices.push_back(pos);

        vertices.push_back(start + size);
        vertices.push_back(pos);
    }

    // diagonales
    for(int i = 0; i < GRID_SIZE; i++)
    {
        for(int j = 0; j < GRID_SIZE; j++)
        {
            float x = start + j * step;
            float y = start + i * step;

            float x2 = x + step;
            float y2 = y + step;

            // diagonal principal
            vertices.push_back(x);
            vertices.push_back(y);

            vertices.push_back(x2);
            vertices.push_back(y2);

            // diagonal inversa
            vertices.push_back(x);
            vertices.push_back(y2);

            vertices.push_back(x2);
            vertices.push_back(y);
        }
    }
}

int main()
{
    glfwInit();

    GLFWwindow* window = glfwCreateWindow(900,900,"Grid 100x100",NULL,NULL);
    glfwMakeContextCurrent(window);

    gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);

    generateGrid();

    GLuint VAO,VBO;

    glGenVertexArrays(1,&VAO);
    glGenBuffers(1,&VBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER,VBO);
    glBufferData(GL_ARRAY_BUFFER,
                 vertices.size()*sizeof(float),
                 vertices.data(),
                 GL_STATIC_DRAW);

    glVertexAttribPointer(0,2,GL_FLOAT,GL_FALSE,2*sizeof(float),(void*)0);
    glEnableVertexAttribArray(0);

    const char* vs =
            "#version 330 core\n"
            "layout(location=0) in vec2 pos;"
            "void main(){gl_Position=vec4(pos,0,1);}";

    const char* fs =
            "#version 330 core\n"
            "out vec4 FragColor;"
            "void main(){FragColor=vec4(1,1,1,1);}";

    GLuint v = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(v,1,&vs,NULL);
    glCompileShader(v);

    GLuint f = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(f,1,&fs,NULL);
    glCompileShader(f);

    GLuint program = glCreateProgram();
    glAttachShader(program,v);
    glAttachShader(program,f);
    glLinkProgram(program);

    while(!glfwWindowShouldClose(window))
    {
        glClearColor(0,0,0,1);
        glClear(GL_COLOR_BUFFER_BIT);

        glUseProgram(program);
        glBindVertexArray(VAO);

        glDrawArrays(GL_LINES,0,vertices.size()/2);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
}