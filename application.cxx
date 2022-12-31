#include <nana/gui.hpp>
#include <nana/gui/widgets/label.hpp>

int main() {
    nana::form fm_;
    nana::label lb_{ fm_, nana::rectangle{ 10, 10, 100, 100} };
    lb_.caption("Hello World!");
    fm_.show();
    nana::exec();
}