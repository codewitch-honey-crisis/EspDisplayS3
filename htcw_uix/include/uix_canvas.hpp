#ifndef HTCW_UIX_CANVAS_HPP
#define HTCW_UIX_CANVAS_HPP
#include <math.h>
#include <bee_icon.hpp>
#include "uix_core.hpp"
namespace uix {
template <typename PixelType, typename PaletteType = gfx::palette<PixelType, PixelType>>
class canvas : public control<PixelType, PaletteType> {
   public:
    using type = canvas;
    using base_type = control<PixelType, PaletteType>;
    using pixel_type = PixelType;
    using palette_type = PaletteType;
    using control_surface_type = typename base_type::control_surface_type;
    typedef void (*on_pressed_changed_callback_type)(bool pressed, void* state);

   private:
    gfx::rgba_pixel<32> m_background_color;
    gfx::svg_doc m_svg;
    canvas(const canvas& rhs) = delete;
    canvas& operator=(const canvas& rhs) = delete;
    void do_move(canvas& rhs) {
        do_move_control(rhs);
        m_background_color = rhs.m_background_color;
        m_svg = helpers::uix_move(rhs.m_svg);
    }
   public:
    canvas(canvas&& rhs) {
        do_move(rhs);
    }
    canvas& operator=(canvas&& rhs) {
        do_move(rhs);
        return *this;
    }

    canvas(invalidation_tracker& parent, const palette_type* palette = nullptr) : base_type(parent, palette) {
        using color_t = gfx::color<gfx::rgba_pixel<32>>;
        background_color(color_t::white);
        gfx::svg_doc::read(&bee_icon,&m_svg);
    }
    gfx::rgba_pixel<32> background_color() const {
        return m_background_color;
    }
    void background_color(gfx::rgba_pixel<32> value) {
        m_background_color = value;
        this->invalidate();
    }
   
    virtual void on_paint(control_surface_type& destination, const srect16& clip) override {
        srect16 b = (srect16)this->dimensions().bounds();
        gfx::draw::svg(destination,this->bounds().dimensions().bounds(),m_svg,m_svg.scale(b.dimensions()));
        base_type::on_paint(destination, clip);
    }
};
}  // namespace uix
#endif