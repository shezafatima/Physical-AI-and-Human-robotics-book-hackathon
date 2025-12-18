import React from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import CourseLayout from '../components/CourseLayout';
import HeroSection from '../components/HeroSection';
import FeaturesSection from '../components/FeaturesSection';
import InteractiveShowcase from '../components/InteractiveShowcase';
import FAQSection from '../components/FAQSection';
import AboutSection from '../components/AboutSection';
import InteractiveQuiz from '../components/InteractiveQuiz';

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <>
      <CourseLayout
        title={`Welcome to ${siteConfig.title}`}
        description="An AI-Native Interactive Coursebook with RAG Chatbot for Physical AI & Humanoid Robotics"
        showChatbot={true}
        isHomePage={true}>
        <HeroSection />
        <FeaturesSection />
        <InteractiveShowcase />
        <FAQSection />
        <AboutSection />
        <InteractiveQuiz />
      </CourseLayout>
    </>
  );
}