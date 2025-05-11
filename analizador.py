import ply.lex as lex
import ply.yacc as yacc

# Tokens
tokens = (
    'NUMERO', 'MAS', 'MENOS', 'MULT', 'DIV',
    'LPAREN', 'RPAREN',
    'IF', 'ELSE', 'WHILE',
)

# Reglas de tokens
t_MAS = r'\+'
t_MENOS = r'-'
t_MULT = r'\*'
t_DIV = r'/'
t_LPAREN = r'\('
t_RPAREN = r'\)'
t_ignore = ' \t'

# Palabras clave
def t_IF(t):
    r'if'
    return t

def t_ELSE(t):
    r'else'
    return t

def t_WHILE(t):
    r'while'
    return t

def t_NUMERO(t):
    r'\d+(\.\d+)?'
    t.value = float(t.value) if '.' in t.value else int(t.value)
    return t

def t_error(t):
    print(f"Carácter ilegal: {t.value[0]}")
    t.lexer.skip(1)

lexer = lex.lex()

# Precedencia
precedence = (
    ('left', 'MAS', 'MENOS'),
    ('left', 'MULT', 'DIV'),
)

# Reglas gramaticales
def p_expresion_binaria(p):
    '''expresion : expresion MAS expresion
                 | expresion MENOS expresion
                 | expresion MULT expresion
                 | expresion DIV expresion'''
    try:
        if p[2] == '+':
            p[0] = p[1] + p[3]
        elif p[2] == '-':
            p[0] = p[1] - p[3]
        elif p[2] == '*':
            p[0] = p[1] * p[3]
        elif p[2] == '/':
            if p[3] == 0:
                print("Error: división por cero")
                p[0] = None
            else:
                p[0] = p[1] / p[3]
    except Exception as e:
        print(f"Error en operación: {e}")
        p[0] = None

def p_expresion_grupo(p):
    'expresion : LPAREN expresion RPAREN'
    p[0] = p[2]

def p_expresion_numero(p):
    'expresion : NUMERO'
    p[0] = p[1]

def p_expresion_if(p):
    'expresion : IF LPAREN expresion RPAREN expresion'
    p[0] = p[5]

def p_expresion_else(p):
    'expresion : ELSE expresion'
    p[0] = p[2]

def p_expresion_while(p):
    'expresion : WHILE LPAREN expresion RPAREN expresion'
    p[0] = p[5]

def p_error(p):
    if p:
        if p.type == 'RPAREN':
            print("Error de sintaxis: paréntesis no balanceados")
        else:
            print(f"Error de sintaxis en '{p.value}'")
    else:
        print("Error de sintaxis: entrada incompleta o paréntesis no balanceados")

parser = yacc.yacc()

if __name__ == "__main__":
    print("Analizador sintáctico (escribe 'salir' para terminar)\n")
    while True:
        try:
            s = input("> ")
            if s.lower() == 'salir':
                break
        except EOFError:
            break
        if not s:
            continue
        resultado = parser.parse(s)
        if resultado is not None:
            print(f"Resultado: {resultado}")