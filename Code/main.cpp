#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_File_Chooser.H>
#include <FL/Fl_Box.H>
#include <FL/Fl_Check_Button.H>
#include <FL/Fl_Progress.H>
#include <FL/Fl_Input.H>
#include <FL/Fl_RGB_Image.H>
#include <fstream>
#include <vector>
#include <iostream>
#include <FL/Fl_Int_Input.H>
#include "Image.h"

using namespace std;

string selected_file;
bool genererSLICC = false;
bool contourSLICC = false;
bool compressSLICC = false;
bool genererMeanShift = false;
bool contourMeanShift = false;
int k = 700;
int m = 55;
float spatial_radius = 10.0f;
float color_radius = 10.0f;
int max_iterations = 100;
Fl_Button* process_button;
Fl_Box* image_box;
Fl_RGB_Image* displayed_image = nullptr; // Global variable to store the image

void update_process_button() {
    bool slicc_ready = genererSLICC && !selected_file.empty();
    bool mean_shift_ready = genererMeanShift && !selected_file.empty() ;
    if (slicc_ready || mean_shift_ready) {
        process_button->activate();
    } else {
        process_button->deactivate();
    }
}

Fl_RGB_Image* loadPPM(const char* filename) {
    ifstream file(filename, ios::binary);
    if (!file) {
        cerr << "Erreur : Impossible d'ouvrir " << filename << endl;
        return nullptr;
    }

    string format;
    int width, height, maxVal;

    file >> format;
    if (format != "P6") {
        cerr << "Format non supporté : " << format << endl;
        return nullptr;
    }



    file >> width >> height >> maxVal;
    file.ignore(); // Sauter le caractère de nouvelle ligne

    if (maxVal != 255) {
        cerr << "Seuls les fichiers avec une plage de 255 sont supportés" << endl;
        return nullptr;
    }

    vector<unsigned char> pixels(width * height * 3);
    file.read(reinterpret_cast<char*>(pixels.data()), pixels.size());

    return new Fl_RGB_Image(pixels.data(), width, height, 3);
}

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
        update_process_button();
    }
}

void process_image(Fl_Widget* w, void* data) {
    if (selected_file.empty()) {
        cout << "No file selected!" << endl;
        return;
    }

    Image img(selected_file, Image::PPM);
    img.read();

    if (genererSLICC) {
        Image imgLAB = img.RGBtoLAB();
        imgLAB.write(selected_file);
        int N = img.getSize();
        //si les paramètres k et m sont vides, on gardes les valeur par défaut, sinon on prendre les valeurs des champs



        imgLAB.SLICC(k, m, N, contourSLICC);
        Image imgOUT = imgLAB.LABtoRGB();
        string nomFichierSortieSLICC = selected_file.substr(0, selected_file.find_last_of('.')) + "_SLICC_" + to_string(k) + "_" + to_string(m) + ".ppm";
        imgOUT.write(nomFichierSortieSLICC);

        if (compressSLICC) {
            Image imgOUTLAB = imgOUT.RGBtoLAB();
            img.genererCourbePSNR(imgLAB, img, k, 10, 50, N);
        }

        if (displayed_image) {
            delete displayed_image; // Delete the previous image to avoid memory leak
        }
        displayed_image = loadPPM(nomFichierSortieSLICC.c_str());
        if (displayed_image) {
            image_box->image(displayed_image);
            image_box->redraw();
        }
    }

    if (genererMeanShift) {
        Image imgLAB = img.RGBtoLAB();
        Image segmentedImg = imgLAB.MeanShiftSegmentation(spatial_radius, color_radius, max_iterations, contourMeanShift);
        Image resultImg = segmentedImg.LABtoRGB();
        string nomFichierSortieMean = selected_file.substr(0, selected_file.find_last_of('.')) + "_MeanShift.ppm";
        resultImg.write(nomFichierSortieMean);

        if (displayed_image) {
            delete displayed_image; // Delete the previous image to avoid memory leak
        }
        displayed_image = loadPPM(nomFichierSortieMean.c_str());
        if (displayed_image) {
            image_box->image(displayed_image);
            image_box->redraw();
        }
    }

    Fl_Progress* progress = (Fl_Progress*)data;
    progress->value(100);
}

void toggle_genererSLICC(Fl_Widget* w, void* data) {
    genererSLICC = !genererSLICC;
    Fl_Input** inputs = (Fl_Input**)data;
    for (int i = 0; i < 2; ++i) {
        if (genererSLICC) {
            inputs[i]->activate();
        } else {
            inputs[i]->deactivate();
        }
    }
    update_process_button();
}

void toggle_genererMeanShift(Fl_Widget* w, void* data) {
    genererMeanShift = !genererMeanShift;
    Fl_Input** inputs = (Fl_Input**)data;
    for (int i = 0; i < 2; ++i) {
        if (genererMeanShift) {
            inputs[i]->activate();
        } else {
            inputs[i]->deactivate();
        }
    }
    update_process_button();
}

int main(int argc, char** argv) {
    Fl_Window* window = new Fl_Window(800, 800, "Image Processing");

    Fl_Button* choose_button = new Fl_Button(10, 10, 100, 30, "Choose Image");
    Fl_Box* file_box = new Fl_Box(120, 10, 470, 30, "No file chosen");

    Fl_Check_Button* slicc_button = new Fl_Check_Button(10, 50, 150, 30, "Generate SLICC");
    Fl_Check_Button* contour_slicc_button = new Fl_Check_Button(10, 90, 150, 30, "Contour SLICC");
    Fl_Check_Button* compress_slicc_button = new Fl_Check_Button(10, 130, 150, 30, "Compress SLICC");

    Fl_Input* k_input = new Fl_Int_Input(300, 50, 100, 30, "k:");

    k_input->deactivate();
    Fl_Input* m_input = new Fl_Int_Input(300, 90, 100, 30, "m:");
    m_input->deactivate();

    Fl_Check_Button* mean_shift_button = new Fl_Check_Button(10, 170, 150, 30, "Generate Mean Shift");
    Fl_Check_Button* contour_mean_shift_button = new Fl_Check_Button(10, 210, 150, 30, "Contour Mean Shift");

    Fl_Input* spatial_input = new Fl_Int_Input(300, 170, 100, 30, "Spatial Radius:");
    spatial_input->deactivate();
    Fl_Input* color_input = new Fl_Int_Input(300, 210, 100, 30, "Color Radius:");
    color_input->deactivate();

    process_button = new Fl_Button(10, 250, 100, 30, "Process Image");
    Fl_Progress* progress = new Fl_Progress(120, 250, 470, 30);
    progress->minimum(0);
    progress->maximum(100);

    image_box = new Fl_Box(10, 300, 780, 480);

    Fl_Input* slicc_inputs[] = {k_input, m_input};
    Fl_Input* mean_shift_inputs[] = {spatial_input, color_input};

    choose_button->callback(choose_file, file_box);
    slicc_button->callback(toggle_genererSLICC, slicc_inputs);
    contour_slicc_button->deactivate();
    compress_slicc_button->deactivate();
    mean_shift_button->callback(toggle_genererMeanShift, mean_shift_inputs);
    contour_mean_shift_button->deactivate();
    process_button->callback(process_image, progress);
    process_button->deactivate();

    k_input->callback([](Fl_Widget* w, void* data) {
        k = atoi(((Fl_Input*)w)->value());
    });
    m_input->callback([](Fl_Widget* w, void* data) {
        m = atoi(((Fl_Input*)w)->value());
    });

    window->end();
    window->show(argc, argv);
    return Fl::run();
}