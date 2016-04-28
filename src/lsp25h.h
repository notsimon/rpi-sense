#pragma once

class Lsp25h {
    int fd_;

public:
    Lsp25h();
    ~Lsp25h();
    float read_pressure() const;
    float read_temp() const;
};
