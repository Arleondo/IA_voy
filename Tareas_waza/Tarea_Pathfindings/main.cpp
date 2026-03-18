#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <vector>
#include <iostream>
#include <cmath>
#include <random>
#include <chrono>

#include "Shader.h"

using namespace std;

// ===== Grilla =====
constexpr int Ancho_grilla = 200;
constexpr int Altura_grilla = 200;

// ===== Zoom =====
float zoom = 1.0f;

// ===== Nodos =====
int nodo1x = -1, nodo1y = -1; // Inicio (verde)
int nodo2x = -1, nodo2y = -1; // Fin (rojo)

// ===== GRID =====
float gridWidth = 1.8f;
float gridHeight = gridWidth * (float)Altura_grilla / (float)Ancho_grilla;

float stepX = gridWidth / Ancho_grilla;
float stepY = gridHeight / Altura_grilla;

float startX = -gridWidth / 2.0f;
float startY = -gridHeight / 2.0f;

// ===== Buffers (renderizado) =====
GLuint VAO, VBO;               // Grilla
GLuint pathVAO, pathVBO;       // A*
GLuint hillVAO, hillVBO;       // Hill climbing
GLuint startNodeVAO, startNodeVBO;
GLuint endNodeVAO, endNodeVBO;

vector<float> vertices;        // Líneas de la grilla
vector<float> pathVertices;
vector<float> hillVertices;

struct Nodo {
    int x, y;
    int nVecinos{};
    Nodo* vecinos[8]{};
    bool bloqueado = false;

    Nodo(int x, int y) : x(x), y(y) {}
};

class Tabla {
public:
    Nodo* grid[Ancho_grilla][Altura_grilla]{};

    Tabla() {
        for (int i = 0; i < Ancho_grilla; i++)
            for (int j = 0; j < Altura_grilla; j++)
                grid[i][j] = new Nodo(i, j);

        for (int i = 0; i < Ancho_grilla; i++)
            for (int j = 0; j < Altura_grilla; j++)
                conectar(i, j);
    }

    void conectar(int x, int y) {
        Nodo* n = grid[x][y];
        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++) {
                if (dx == 0 && dy == 0) continue;
                int nx = x + dx;
                int ny = y + dy;
                if (nx >= 0 && nx < Ancho_grilla && ny >= 0 && ny < Altura_grilla)
                    n->vecinos[n->nVecinos++] = grid[nx][ny];
            }
    }
};

// ===== Heuristica =====
float H(Nodo* a, Nodo* b) {
    return abs(a->x - b->x) + abs(a->y - b->y);
}

// A*
void AStar(Tabla& t) {
    pathVertices.clear();
    if (nodo1x == -1 || nodo2x == -1) return;

    Nodo* start = t.grid[nodo1x][nodo1y];
    Nodo* end = t.grid[nodo2x][nodo2y];

    if (start->bloqueado || end->bloqueado) return;

    bool closed[Ancho_grilla][Altura_grilla]{};
    float g[Ancho_grilla][Altura_grilla];
    float f[Ancho_grilla][Altura_grilla];
    Nodo* parent[Ancho_grilla][Altura_grilla]{};

    for (int i = 0; i < Ancho_grilla; i++)
        for (int j = 0; j < Altura_grilla; j++)
            g[i][j] = f[i][j] = 999999;

    g[start->x][start->y] = 0;
    f[start->x][start->y] = H(start, end);

    while (true) {
        Nodo* current = nullptr;
        float best = 999999;

        for (int i = 0; i < Ancho_grilla; i++)
            for (int j = 0; j < Altura_grilla; j++)
                if (!closed[i][j] && f[i][j] < best) {
                    best = f[i][j];
                    current = t.grid[i][j];
                }

        if (!current) return;
        if (current == end) break;

        closed[current->x][current->y] = true;

        for (int i = 0; i < current->nVecinos; i++) {
            Nodo* nb = current->vecinos[i];
            if (nb->bloqueado) continue;
            if (closed[nb->x][nb->y]) continue;

            float newG = g[current->x][current->y] + 1;

            if (newG < g[nb->x][nb->y]) {
                parent[nb->x][nb->y] = current;
                g[nb->x][nb->y] = newG;
                f[nb->x][nb->y] = newG + H(nb, end);
            }
        }
    }

    Nodo* c = end;
    while (c != start) {
        Nodo* p = parent[c->x][c->y];
        float x1 = startX + c->x * stepX;
        float y1 = startY + c->y * stepY;
        float x2 = startX + p->x * stepX;
        float y2 = startY + p->y * stepY;
        pathVertices.insert(pathVertices.end(), { x1, y1, x2, y2 });
        c = p;
    }
}

// Hill Climbing
void Hill(Tabla& t) {
    hillVertices.clear();
    if (nodo1x == -1 || nodo2x == -1) return;

    Nodo* start = t.grid[nodo1x][nodo1y];
    Nodo* end = t.grid[nodo2x][nodo2y];

    if (start->bloqueado || end->bloqueado) return;

    // Lista L
    vector<Nodo*> L;
    vector<vector<Nodo*>> caminos; // guarda el camino hasta cada nodo

    L.push_back(start);
    caminos.push_back({ start });

    while (!L.empty()) {
        // Tomar el primero (mejor)
        Nodo* n = L.front();
        vector<Nodo*> camino = caminos.front();

        // Paso 3: ¿es objetivo?
        if (n == end) {
            // Dibujar camino
            for (int i = 0; i < (int)camino.size() - 1; i++) {
                Nodo* a = camino[i];
                Nodo* b = camino[i + 1];

                float x1 = startX + a->x * stepX;
                float y1 = startY + a->y * stepY;
                float x2 = startX + b->x * stepX;
                float y2 = startY + b->y * stepY;

                hillVertices.insert(hillVertices.end(), { x1, y1, x2, y2 });
            }
            return;
        }

        // Remover n de L
        L.erase(L.begin());
        caminos.erase(caminos.begin());

        // Obtener hijos válidos
        vector<Nodo*> hijos;
        for (int i = 0; i < n->nVecinos; i++) {
            Nodo* nb = n->vecinos[i];
            if (!nb->bloqueado)
                hijos.push_back(nb);
        }

        // Ordenar hijos por heurística (menor primero)
        sort(hijos.begin(), hijos.end(), [&](Nodo* a, Nodo* b) {
            return H(a, end) < H(b, end);
        });

        // Insertar hijos al inicio de L
        for (int i = hijos.size() - 1; i >= 0; i--) {
            Nodo* h = hijos[i];

            // Crear nuevo camino
            vector<Nodo*> nuevoCamino = camino;
            nuevoCamino.push_back(h);

            L.insert(L.begin(), h);
            caminos.insert(caminos.begin(), nuevoCamino);
        }
    }

    // Si L queda vacío → no hay solución
}

// regenerar grilla
void regenerarGrilla(Tabla& t) {
    vertices.clear();
    for (int i = 0; i < Ancho_grilla; i++) {
        for (int j = 0; j < Altura_grilla; j++) {
            if (t.grid[i][j]->bloqueado) continue;

            float x = startX + i * stepX;
            float y = startY + j * stepY;

            // Derecha
            if (i < Ancho_grilla - 1 && !t.grid[i+1][j]->bloqueado) {
                vertices.push_back(x); vertices.push_back(y);
                vertices.push_back(x + stepX); vertices.push_back(y);
            }
            // Arriba
            if (j < Altura_grilla - 1 && !t.grid[i][j+1]->bloqueado) {
                vertices.push_back(x); vertices.push_back(y);
                vertices.push_back(x); vertices.push_back(y + stepY);
            }
            // Diagonal arriba-derecha
            if (i < Ancho_grilla - 1 && j < Altura_grilla - 1 && !t.grid[i+1][j+1]->bloqueado) {
                vertices.push_back(x); vertices.push_back(y);
                vertices.push_back(x + stepX); vertices.push_back(y + stepY);
            }
            // Diagonal abajo-derecha
            if (i < Ancho_grilla - 1 && j > 0 && !t.grid[i+1][j-1]->bloqueado) {
                vertices.push_back(x); vertices.push_back(y);
                vertices.push_back(x + stepX); vertices.push_back(y - stepY);
            }
        }
    }

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_DYNAMIC_DRAW);
}

// Borrado del 20%
void bloquearNodosAleatorios(Tabla& t) {
    unsigned seed = chrono::high_resolution_clock::now().time_since_epoch().count();
    mt19937 gen(seed);
    uniform_real_distribution<float> dist(0.0f, 1.0f);

    for (int i = 0; i < Ancho_grilla; i++)
        for (int j = 0; j < Altura_grilla; j++)
            t.grid[i][j]->bloqueado = (dist(gen) < 0.2f);

    if (nodo1x != -1 && t.grid[nodo1x][nodo1y]->bloqueado) {
        nodo1x = nodo1y = -1;
        glBindBuffer(GL_ARRAY_BUFFER, startNodeVBO);
        glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
    }
    if (nodo2x != -1 && t.grid[nodo2x][nodo2y]->bloqueado) {
        nodo2x = nodo2y = -1;
        glBindBuffer(GL_ARRAY_BUFFER, endNodeVBO);
        glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
    }

    pathVertices.clear();
    hillVertices.clear();
    regenerarGrilla(t);
}

// Seleccionado
bool getNode(GLFWwindow* w, int& gx, int& gy) {
    double mx, my;
    glfwGetCursorPos(w, &mx, &my);

    int width, height;
    glfwGetWindowSize(w, &width, &height);

    // NDC
    float ndcX = (2.0f * mx) / width - 1.0f;
    float ndcY = 1.0f - (2.0f * my) / height;

    float worldX = ndcX / zoom;
    float worldY = ndcY / zoom;

    // Grid
    float fx = (worldX - startX) / stepX;
    float fy = (worldY - startY) / stepY;

    gx = (int)fx;
    gy = (int)fy;

    return (gx >= 0 && gx < Ancho_grilla && gy >= 0 && gy < Altura_grilla);
}

// scrooll
void scroll(GLFWwindow*, double, double y) {
    zoom += y * 0.1f;
    if (zoom < 0.2f) zoom = 0.2f;
    if (zoom > 10.0f) zoom = 10.0f;
}

// ===== init de buffers =====
void initBuffers() {
    glGenVertexArrays(1, &VAO); glGenBuffers(1, &VBO);
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    glGenVertexArrays(1, &pathVAO); glGenBuffers(1, &pathVBO);
    glBindVertexArray(pathVAO);
    glBindBuffer(GL_ARRAY_BUFFER, pathVBO);
    glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    glGenVertexArrays(1, &hillVAO); glGenBuffers(1, &hillVBO);
    glBindVertexArray(hillVAO);
    glBindBuffer(GL_ARRAY_BUFFER, hillVBO);
    glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    glGenVertexArrays(1, &startNodeVAO); glGenBuffers(1, &startNodeVBO);
    glBindVertexArray(startNodeVAO);
    glBindBuffer(GL_ARRAY_BUFFER, startNodeVBO);
    glBufferData(GL_ARRAY_BUFFER, 2 * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    glGenVertexArrays(1, &endNodeVAO); glGenBuffers(1, &endNodeVBO);
    glBindVertexArray(endNodeVAO);
    glBindBuffer(GL_ARRAY_BUFFER, endNodeVBO);
    glBufferData(GL_ARRAY_BUFFER, 2 * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}


int main() {
    if (!glfwInit()) return -1;

    GLFWmonitor* monitor = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(monitor);
    GLFWwindow* window = glfwCreateWindow(mode->width, mode->height, "A* Pathfinding", monitor, NULL);
    if (!window) { glfwTerminate(); return -1; }

    glfwMakeContextCurrent(window);
    glfwSetScrollCallback(window, scroll);
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) return -1;

    glEnable(GL_PROGRAM_POINT_SIZE);
    glPointSize(10.0f);

    Shader shader("vertex.glsl", "fragment.glsl");

    Tabla tabla;
    initBuffers();
    regenerarGrilla(tabla); // Grilla inicial sin bloqueos

    bool backspacePressed = false;

    while (!glfwWindowShouldClose(window)) {
        // Input
        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
            glfwSetWindowShouldClose(window, true);

        // Backspace: bloquear 20%
        if (glfwGetKey(window, GLFW_KEY_BACKSPACE) == GLFW_PRESS) {
            if (!backspacePressed) {
                bloquearNodosAleatorios(tabla);
                backspacePressed = true;
            }
        } else {
            backspacePressed = false;
        }

        // Tecla 1: A* (azul)
        if (glfwGetKey(window, GLFW_KEY_1) == GLFW_PRESS) {
            AStar(tabla);
        }
        // Tecla 2: Hill (naranja)
        if (glfwGetKey(window, GLFW_KEY_2) == GLFW_PRESS) {
            Hill(tabla);
        }

        // Selección de nodos
        if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
            int x, y;
            if (getNode(window, x, y) && !tabla.grid[x][y]->bloqueado) {
                nodo1x = x; nodo1y = y;
                float pos[2] = { startX + x * stepX, startY + y * stepY };
                glBindBuffer(GL_ARRAY_BUFFER, startNodeVBO);
                glBufferData(GL_ARRAY_BUFFER, sizeof(pos), pos, GL_DYNAMIC_DRAW);
            }
        }

        if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) {
            int x, y;
            if (getNode(window, x, y) && !tabla.grid[x][y]->bloqueado) {
                nodo2x = x; nodo2y = y;
                float pos[2] = { startX + x * stepX, startY + y * stepY };
                glBindBuffer(GL_ARRAY_BUFFER, endNodeVBO);
                glBufferData(GL_ARRAY_BUFFER, sizeof(pos), pos, GL_DYNAMIC_DRAW);
            }
        }

        // Render
        int width, height;
        glfwGetWindowSize(window, &width, &height);
        float aspect = (float)width / (float)height;

        glClear(GL_COLOR_BUFFER_BIT);

        shader.use();
        shader.setFloat("zoom", zoom);
        shader.setFloat("aspect", aspect);
        shader.setVec2("camera", 0.0f, 0.0f);

        // Grilla blanca
        shader.setVec3("color", 1.0f, 1.0f, 1.0f);
        glBindVertexArray(VAO);
        glDrawArrays(GL_LINES, 0, vertices.size() / 2);

        // A* azul
        if (!pathVertices.empty()) {
            glBindVertexArray(pathVAO);
            glBindBuffer(GL_ARRAY_BUFFER, pathVBO);
            glBufferData(GL_ARRAY_BUFFER, pathVertices.size() * sizeof(float), pathVertices.data(), GL_DYNAMIC_DRAW);
            shader.setVec3("color", 0.0f, 0.0f, 1.0f);
            glDrawArrays(GL_LINES, 0, pathVertices.size() / 2);
        }

        // Hill naranja
        if (!hillVertices.empty()) {
            glBindVertexArray(hillVAO);
            glBindBuffer(GL_ARRAY_BUFFER, hillVBO);
            glBufferData(GL_ARRAY_BUFFER, hillVertices.size() * sizeof(float), hillVertices.data(), GL_DYNAMIC_DRAW);
            shader.setVec3("color", 1.0f, 0.5f, 0.0f);
            glDrawArrays(GL_LINES, 0, hillVertices.size() / 2);
        }

        // Nodo inicio verde
        if (nodo1x != -1) {
            glBindVertexArray(startNodeVAO);
            shader.setVec3("color", 0.0f, 1.0f, 0.0f);
            glDrawArrays(GL_POINTS, 0, 1);
        }

        // Nodo fin rojo
        if (nodo2x != -1) {
            glBindVertexArray(endNodeVAO);
            shader.setVec3("color", 1.0f, 0.0f, 0.0f);
            glDrawArrays(GL_POINTS, 0, 1);
        }

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}