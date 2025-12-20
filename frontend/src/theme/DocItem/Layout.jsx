import React from 'react';
import OriginalDocItemLayout from '@theme-original/DocItem/Layout';
import CourseLayout from '../../components/CourseLayout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

export default function DocItemLayout(props) {
  const { siteConfig } = useDocusaurusContext();
  const { content: docContent } = props;
  const { metadata } = docContent || {};
  const title = metadata?.title || siteConfig.title || "Course Content";
  const description = metadata?.description || siteConfig.tagline || "Interactive coursebook with RAG chatbot";

  // Use CourseLayout as a content wrapper for documentation pages
  return (
    <CourseLayout
      title={title}
      description={description}
      // showChatbot={true}
      isHomePage={false}
    >
      <OriginalDocItemLayout {...props} />
    </CourseLayout>
  );
}