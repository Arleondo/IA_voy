#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <vector>
#include <iostream>

using namespace std;

constexpr int Ancho_grilla  = 100;
constexpr int Altura_grilla = 70;

struct Nodo {
    int coordx, coordy;
    int Numero_vecinos{};
    Nodo* vecinos[8]{};

    Nodo(int x,int y):coordx(x),coordy(y){}
};

class Tabla {

public:

    Nodo* grilla[Ancho_grilla][Altura_grilla]{};

    Tabla()
    {
        for(int i=0;i<Ancho_grilla;i++)
        {
            for(int j=0;j<Altura_grilla;j++)
            {
                grilla[i][j] = new Nodo(i,j);
            }
        }

        for(int i=0;i<Ancho_grilla;i++)
        {
            for(int j=0;j<Altura_grilla;j++)
            {
                conectarVecinos(i,j);
            }
        }
    }

    void conectarVecinos(int x,int y) const {
        Nodo* nodo = grilla[x][y];

        for(int dx=-1;dx<=1;dx++)
        {
            for(int dy=-1;dy<=1;dy++)
            {
                if(dx==0 && dy==0) continue;

                int nx = x + dx;
                int ny = y + dy;

                if(nx>=0 && nx<Ancho_grilla && ny>=0 && ny<Altura_grilla)
                {
                    nodo->vecinos[nodo->Numero_vecinos++] = grilla[nx][ny];
                }
            }
        }
    }

};

vector<float> vertices;

int nodo1x=-1,nodo1y=-1;
int nodo2x=-1,nodo2y=-1;

float gridWidth = 1.8f;
float gridHeight = gridWidth * static_cast<float>(Altura_grilla) / static_cast<float>(Ancho_grilla);

float stepX = gridWidth / Ancho_grilla;
float stepY = gridHeight / Altura_grilla;

float startX = -gridWidth/2.0f;
float startY = -gridHeight/2.0f;

vector<float> pathVertices;

float heuristica(Nodo* a, Nodo* b)
{
    return abs(a->coordx - b->coordx) + abs(a->coordy - b->coordy);
}

void calcularAStar(Tabla& tabla)
{
    pathVertices.clear();

    if(nodo1x == -1 || nodo2x == -1)
    {
        cout<<"ERROR: selecciona nodo inicio y final\n";
        return;
    }

    Nodo* inicio = tabla.grilla[nodo1x][nodo1y];
    Nodo* fin    = tabla.grilla[nodo2x][nodo2y];

    bool cerrado[Ancho_grilla][Altura_grilla]{};
    float g[Ancho_grilla][Altura_grilla];
    float f[Ancho_grilla][Altura_grilla];

    Nodo* padre[Ancho_grilla][Altura_grilla]{};

    for(int i=0;i<Ancho_grilla;i++)
    {
        for(int j=0;j<Altura_grilla;j++)
        {
            g[i][j] = 999999.0f;
            f[i][j] = 999999.0f;
        }
    }

    g[inicio->coordx][inicio->coordy] = 0;
    f[inicio->coordx][inicio->coordy] = heuristica(inicio,fin);

    while(true)
    {
        Nodo* actual = nullptr;
        float mejorF = 999999;

        for(int i=0;i<Ancho_grilla;i++)
        {
            for(int j=0;j<Altura_grilla;j++)
            {
                if(!cerrado[i][j] && f[i][j] < mejorF)
                {
                    mejorF = f[i][j];
                    actual = tabla.grilla[i][j];
                }
            }
        }

        if(actual == nullptr)
        {
            cout<<"No se encontro camino\n";
            return;
        }

        if(actual == fin)
        {
            cout<<"Camino encontrado\n";
            break;
        }

        cerrado[actual->coordx][actual->coordy] = true;

        for(int v=0; v<actual->Numero_vecinos; v++)
        {
            Nodo* vecino = actual->vecinos[v];

            if(cerrado[vecino->coordx][vecino->coordy])
                continue;

            float nuevoG = g[actual->coordx][actual->coordy] + 1;

            if(nuevoG < g[vecino->coordx][vecino->coordy])
            {
                padre[vecino->coordx][vecino->coordy] = actual;

                g[vecino->coordx][vecino->coordy] = nuevoG;

                f[vecino->coordx][vecino->coordy] =
                        nuevoG + heuristica(vecino,fin);
            }
        }
    }

    Nodo* actual = fin;

    while(actual != inicio)
    {
        Nodo* p = padre[actual->coordx][actual->coordy];

        float x1 = startX + actual->coordx * stepX;
        float y1 = startY + actual->coordy * stepY;

        float x2 = startX + p->coordx * stepX;
        float y2 = startY + p->coordy * stepY;

        pathVertices.push_back(x1);
        pathVertices.push_back(y1);

        pathVertices.push_back(x2);
        pathVertices.push_back(y2);

        actual = p;
    }
}

vector<float> generarVertices(const Tabla& tabla)
{
    vector<float> verts;

    for(int i=0;i<Ancho_grilla;i++)
    {
        for(int j=0;j<Altura_grilla;j++)
        {
            Nodo* nodo = tabla.grilla[i][j];

            float x1 = startX + nodo->coordx * stepX;
            float y1 = startY + nodo->coordy * stepY;

            for(int v=0; v<nodo->Numero_vecinos; v++)
            {
                Nodo* vecino = nodo->vecinos[v];

                if(vecino->coordx < nodo->coordx) continue;
                if(vecino->coordx == nodo->coordx && vecino->coordy <= nodo->coordy) continue;

                float x2 = startX + vecino->coordx * stepX;
                float y2 = startY + vecino->coordy * stepY;

                verts.push_back(x1);
                verts.push_back(y1);

                verts.push_back(x2);
                verts.push_back(y2);
            }
        }
    }

    return verts;
}

bool obtenerNodoDesdeMouse(GLFWwindow* window,int& gx,int& gy)
{
    double xpos,ypos;
    glfwGetCursorPos(window,&xpos,&ypos);

    int width,height;
    glfwGetWindowSize(window,&width,&height);

    float nx = (xpos/width)*2.0f - 1.0f;
    float ny = -((ypos/height)*2.0f - 1.0f);

    float best = 1000.0f;

    for(int i=0;i<Ancho_grilla;i++)
    {
        for(int j=0;j<Altura_grilla;j++)
        {
            float cx = startX + i*stepX;
            float cy = startY + j*stepY;

            float dx = nx-cx;
            float dy = ny-cy;

            float dist = dx*dx + dy*dy;

            if(dist < best)
            {
                best = dist;
                gx = i;
                gy = j;
            }
        }
    }

    return true;
}

void mouse_callback(GLFWwindow* window,int button,int action,int mods)
{
    if(action!=GLFW_PRESS) return;

    int gx,gy;

    if(!obtenerNodoDesdeMouse(window,gx,gy)) return;

    if(button==GLFW_MOUSE_BUTTON_LEFT)
    {
        nodo1x=gx;
        nodo1y=gy;
        cout<<"Nodo rojo: "<<gx<<" "<<gy<<endl;
    }

    if(button==GLFW_MOUSE_BUTTON_RIGHT)
    {
        nodo2x=gx;
        nodo2y=gy;
        cout<<"Nodo azul: "<<gx<<" "<<gy<<endl;
    }
}

GLuint crearShader(GLenum type,const char* src)
{
    GLuint s = glCreateShader(type);
    glShaderSource(s,1,&src,nullptr);
    glCompileShader(s);
    return s;
}

int main()
{
    glfwInit();

    GLFWmonitor* monitor = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(monitor);

    GLFWwindow* window = glfwCreateWindow(
            mode->width,
            mode->height,
            "Grid",
            monitor,
            nullptr
    );

    glfwMakeContextCurrent(window);

    gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress));

    glViewport(0,0,mode->width,mode->height);

    glfwSetMouseButtonCallback(window,mouse_callback);

    Tabla tabla;

    vertices = generarVertices(tabla);

    GLuint VAO,VBO;

    glGenVertexArrays(1,&VAO);
    glGenBuffers(1,&VBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER,VBO);
    glBufferData(GL_ARRAY_BUFFER,
                 vertices.size()*sizeof(float),
                 vertices.data(),
                 GL_STATIC_DRAW);

    glVertexAttribPointer(0,2,GL_FLOAT,GL_FALSE,2*sizeof(float),static_cast<void *>(nullptr));
    glEnableVertexAttribArray(0);

    const char* vs=
            "#version 330 core\n"
            "layout(location=0) in vec2 pos;"
            "void main(){gl_Position=vec4(pos,0,1);}";

    const char* fs=
            "#version 330 core\n"
            "uniform vec3 color;"
            "out vec4 FragColor;"
            "void main(){FragColor=vec4(color,1);}";

    GLuint v=crearShader(GL_VERTEX_SHADER,vs);
    GLuint f=crearShader(GL_FRAGMENT_SHADER,fs);

    GLuint program=glCreateProgram();
    glAttachShader(program,v);
    glAttachShader(program,f);
    glLinkProgram(program);

    GLuint pointVAO,pointVBO;

    glGenVertexArrays(1,&pointVAO);
    glGenBuffers(1,&pointVBO);

    glBindVertexArray(pointVAO);
    glBindBuffer(GL_ARRAY_BUFFER,pointVBO);
    glBufferData(GL_ARRAY_BUFFER,2*sizeof(float),nullptr,GL_DYNAMIC_DRAW);

    glVertexAttribPointer(0,2,GL_FLOAT,GL_FALSE,2*sizeof(float),static_cast<void *>(nullptr));
    glEnableVertexAttribArray(0);

    glPointSize(12.0f);


    while(!glfwWindowShouldClose(window))
    {
        if(glfwGetKey(window,GLFW_KEY_ESCAPE)==GLFW_PRESS)
            glfwSetWindowShouldClose(window,true);

        glClearColor(0,0,0,1);
        glClear(GL_COLOR_BUFFER_BIT);

        glUseProgram(program);

        glUniform3f(glGetUniformLocation(program,"color"),1,1,1);

        glBindVertexArray(VAO);
        glDrawArrays(GL_LINES,0,vertices.size()/2);

        if(!pathVertices.empty()){
            GLuint pathVBO, pathVAO;

            glGenVertexArrays(1,&pathVAO);
            glGenBuffers(1,&pathVBO);

            glBindVertexArray(pathVAO);
            glBindBuffer(GL_ARRAY_BUFFER,pathVBO);

            glBufferData(GL_ARRAY_BUFFER,
                         pathVertices.size()*sizeof(float),
                         pathVertices.data(),
                         GL_STATIC_DRAW);

            glVertexAttribPointer(0,2,GL_FLOAT,GL_FALSE,2*sizeof(float),(void*)0);
            glEnableVertexAttribArray(0);

            glUniform3f(glGetUniformLocation(program,"color"),0,1,0);

            glDrawArrays(GL_LINES,0,pathVertices.size()/2);

            glDeleteBuffers(1,&pathVBO);
            glDeleteVertexArrays(1,&pathVAO);
        }

        if(nodo1x!=-1){
            float p[2] =
            {
                startX + nodo1x*stepX,
                startY + nodo1y*stepY
            };

            glBindBuffer(GL_ARRAY_BUFFER,pointVBO);
            glBufferSubData(GL_ARRAY_BUFFER,0,sizeof(p),p);

            glUniform3f(glGetUniformLocation(program,"color"),1,0,0);

            glBindVertexArray(pointVAO);
            glDrawArrays(GL_POINTS,0,1);
        }

        if(nodo2x!=-1){
            float p[2] ={
                startX + nodo2x*stepX,
                startY + nodo2y*stepY
            };

            glBindBuffer(GL_ARRAY_BUFFER,pointVBO);
            glBufferSubData(GL_ARRAY_BUFFER,0,sizeof(p),p);

            glUniform3f(glGetUniformLocation(program,"color"),0,0,1);

            glBindVertexArray(pointVAO);
            glDrawArrays(GL_POINTS,0,1);
        }

        if(glfwGetKey(window,GLFW_KEY_1) == GLFW_PRESS){
            calcularAStar(tabla);
        }

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
}