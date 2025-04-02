#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_File_Chooser.H>
#include <FL/Fl_Box.H>
#include <FL/Fl_Check_Button.H>
#include <FL/Fl_Progress.H>
#include <iostream>
#include "Image.h"

using namespace std;

string selected_file;
bool genererSLICC = false;
bool contourSLICC = false;
bool compressSLICC = false;
bool genererMeanShift = false;
bool contourMeanShift = false;

void choose_file(Fl_Widget* w, void* data) {
    Fl_File_Chooser chooser(".", "*.{ppm}", Fl_File_Chooser::SINGLE, "Choose an Image");
    chooser.show();
    while (chooser.shown()) {
        Fl::wait();
    }
    if (chooser.value() != nullptr) {
        selected_file = chooser.value();
        Fl_Box* box = (Fl_Box*)data;
        box->label(selected_file.c_str());
    }
}

void process_image(Fl_Widget* w, void* data) {
    if (selected_file.empty()) {
        cout << "No file selected!" << endl;
        return;
    }

    //si l'image n'est pas au format PPM, on la convertit
    if (selected_file.substr(selected_file.find_last_of('.') + 1) != "ppm") {
        cout << "Image is not in PPM format. Converting..." << endl;
        // Convert the image to PPM format
        Image img(selected_file, Image::PPM);
        img.read();
        string converted_file = selected_file.substr(0, selected_file.find_last_of('.')) + ".ppm";
        img.write(converted_file);
        selected_file = converted_file;
    }

    Image img(selected_file, Image::PPM);
    img.read();

    if (genererSLICC) {
        Image imgLAB = img.RGBtoLAB();
        imgLAB.write(selected_file);
        int k = 700; // Number of clusters
        int m = 55; // Spatial resolution
        int N = img.getSize();
        imgLAB.SLICC(k, m, N, contourSLICC);
        Image imgOUT = imgLAB.LABtoRGB();
        string nomFichierSortieSLICC = selected_file.substr(0, selected_file.find_last_of('.')) + "_SLICC_" + to_string(k) + "_" + to_string(m) + ".ppm";
        imgOUT.write(nomFichierSortieSLICC);

        if (compressSLICC) {
            Image imgOUTLAB = imgOUT.RGBtoLAB();
            img.genererCourbePSNR(imgLAB, img, k, 10, 50, N);
        }
    }

    if (genererMeanShift) {
        float spatial_radius = 10.0f;
        float color_radius = 10.0f;
        int max_iterations = 100;

        Image imgLAB = img.RGBtoLAB();
        Image segmentedImg = imgLAB.MeanShiftSegmentation(spatial_radius, color_radius, max_iterations, contourMeanShift);
        Image resultImg = segmentedImg.LABtoRGB();
        string nomFichierSortieMean = selected_file.substr(0, selected_file.find_last_of('.')) + "_MeanShift.ppm";
        resultImg.write(nomFichierSortieMean);
    }

    Fl_Progress* progress = (Fl_Progress*)data;
    progress->value(100);
}

void toggle_genererSLICC(Fl_Widget* w, void*) {
    genererSLICC = !genererSLICC;
}

void toggle_contourSLICC(Fl_Widget* w, void*) {
    contourSLICC = !contourSLICC;
}

void toggle_compressSLICC(Fl_Widget* w, void*) {
    compressSLICC = !compressSLICC;
}

void toggle_genererMeanShift(Fl_Widget* w, void*) {
    genererMeanShift = !genererMeanShift;
}

void toggle_contourMeanShift(Fl_Widget* w, void*) {
    contourMeanShift = !contourMeanShift;
}

int main(int argc, char** argv) {
    Fl_Window* window = new Fl_Window(400, 400, "Image Processing");

    Fl_Button* choose_button = new Fl_Button(10, 10, 100, 30, "Choose Image");
    Fl_Box* file_box = new Fl_Box(120, 10, 270, 30, "No file chosen");

    Fl_Check_Button* slicc_button = new Fl_Check_Button(10, 50, 150, 30, "Generate SLICC");
    Fl_Check_Button* contour_slicc_button = new Fl_Check_Button(10, 90, 150, 30, "Contour SLICC");
    Fl_Check_Button* compress_slicc_button = new Fl_Check_Button(10, 130, 150, 30, "Compress SLICC");

    Fl_Check_Button* mean_shift_button = new Fl_Check_Button(10, 170, 150, 30, "Generate Mean Shift");
    Fl_Check_Button* contour_mean_shift_button = new Fl_Check_Button(10, 210, 150, 30, "Contour Mean Shift");

    Fl_Button* process_button = new Fl_Button(10, 250, 100, 30, "Process Image");
    Fl_Progress* progress = new Fl_Progress(120, 250, 270, 30);
    progress->minimum(0);
    progress->maximum(100);

    choose_button->callback(choose_file, file_box);
    slicc_button->callback(toggle_genererSLICC);
    contour_slicc_button->callback(toggle_contourSLICC);
    compress_slicc_button->callback(toggle_compressSLICC);
    mean_shift_button->callback(toggle_genererMeanShift);
    contour_mean_shift_button->callback(toggle_contourMeanShift);
    process_button->callback(process_image, progress);

    window->end();
    window->show(argc, argv);
    return Fl::run();
}