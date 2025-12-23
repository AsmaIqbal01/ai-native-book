/**
 * Content Translation Mappings
 *
 * Comprehensive Urdu translations for book content, headings, and common phrases.
 * This file contains static translations that are loaded into the TranslationService.
 *
 * Structure:
 * - Keys: Original English text
 * - Values: Urdu translation
 *
 * Usage:
 * import { contentTranslations } from './translations/contentTranslations';
 * TranslationService.addTranslations(contentTranslations, 'ur');
 */

export const contentTranslations: Record<string, string> = {
  // ==================== Chapter Titles ====================
  'Chapter 1: Introduction to Physical AI': 'باب 1: فزیکل AI کا تعارف',
  'Chapter 2: ROS2 Fundamentals': 'باب 2: ROS2 کی بنیادی باتیں',
  'Chapter 3: Simulation Environments': 'باب 3: نقلی ماحول',
  'Chapter 4: Vision-Language-Action Models': 'باب 4: Vision-Language-Action ماڈلز',
  'Chapter 5: Advanced Topics': 'باب 5: جدید موضوعات',
  'Resources': 'وسائل',

  // ==================== Section Headings ====================
  'Learning Objectives': 'سیکھنے کے مقاصد',
  'By the end of this section, you will:': 'اس حصے کے اختتام تک، آپ:',
  'Prerequisites': 'پیشگی ضروریات',
  'Introduction': 'تعارف',
  'Overview': 'جائزہ',
  'Summary': 'خلاصہ',
  'Key Takeaways': 'کلیدی نکات',
  'Next Steps': 'اگلے مراحل',
  'Further Reading': 'مزید مطالعہ',
  'Table of Contents': 'فہرست',

  // ==================== Common Content Phrases ====================
  'What is': 'کیا ہے',
  'Why': 'کیوں',
  'How': 'کیسے',
  'When': 'کب',
  'Where': 'کہاں',
  'Example': 'مثال',
  'Examples': 'مثالیں',
  'Note': 'نوٹ',
  'Important': 'اہم',
  'Warning': 'انتباہ',
  'Tip': 'مشورہ',
  'Caution': 'احتیاط',
  'Info': 'معلومات',

  // ==================== Physical AI Content ====================
  'The Humanoid Robotics Landscape': 'ہیومنائڈ روبوٹکس کا منظرنامہ',
  'Exploring current humanoid robot platforms, companies, and technical approaches':
    'موجودہ ہیومنائڈ روبوٹ پلیٹ فارمز، کمپنیوں، اور تکنیکی طریقوں کی تلاش',

  // Learning Objectives
  'Understand the current state of humanoid robotics technology':
    'ہیومنائڈ روبوٹکس ٹیکنالوجی کی موجودہ صورتحال کو سمجھیں',
  'Learn about major humanoid robot platforms and their capabilities':
    'اہم ہیومنائڈ روبوٹ پلیٹ فارمز اور ان کی صلاحیتوں کے بارے میں جانیں',
  'Explore different design philosophies and technical approaches':
    'مختلف ڈیزائن فلسفوں اور تکنیکی طریقوں کو دریافت کریں',
  'Recognize key hardware components and architectures':
    'کلیدی ہارڈویئر اجزاء اور فن تعمیر کو پہچانیں',
  'Identify current limitations and future directions':
    'موجودہ حدود اور مستقبل کی سمتوں کی نشاندہی کریں',

  // Why Humanoid Robots?
  'Why Humanoid Robots?': 'ہیومنائڈ روبوٹس کیوں؟',
  "The humanoid form factor isn't just about making robots look like us—it's a practical engineering choice:":
    'ہیومنائڈ شکل صرف روبوٹس کو ہماری طرح بنانے کے بارے میں نہیں ہے — یہ ایک عملی انجینئرنگ کا انتخاب ہے:',

  'Environmental Compatibility': 'ماحولیاتی مطابقت',
  'Our world is designed for human bodies. Humanoid robots can:':
    'ہماری دنیا انسانی جسموں کے لیے ڈیزائن کی گئی ہے۔ ہیومنائڈ روبوٹس کر سکتے ہیں:',
  'Navigate stairs, doorways, and narrow spaces':
    'سیڑھیاں، دروازے، اور تنگ جگہوں پر چلنا',
  'Use tools designed for human hands': 'انسانی ہاتھوں کے لیے ڈیزائن کیے گئے اوزار استعمال کرنا',
  'Operate vehicles and machinery': 'گاڑیاں اور مشینری چلانا',
  'Work in existing infrastructure without modifications':
    'موجودہ بنیادی ڈھانچے میں بغیر تبدیلیوں کے کام کرنا',

  'Intuitive Interaction': 'بدیہی تعامل',
  'Human-like form enables:': 'انسانی شکل ممکن بناتی ہے:',
  'Natural communication through gestures and body language':
    'اشاروں اور جسمانی زبان کے ذریعے قدرتی بات چیت',
  'Predictable movement patterns for human collaborators':
    'انسانی ساتھیوں کے لیے قابل پیش گوئی حرکت کے نمونے',
  'Social acceptance in shared spaces': 'مشترکہ جگہوں میں سماجی قبولیت',
  'Easier teaching through demonstration': 'مظاہرے کے ذریعے آسان تعلیم',

  // Admonitions
  'Engineering Insight': 'انجینئرنگ بصیرت',
  'The humanoid form factor represents a trade-off: increased complexity for universal adaptability.':
    'ہیومنائڈ فارم فیکٹر ایک تجارتی تبادلہ کی نمائندگی کرتا ہے: عالمگیر موافقت کے لیے بڑھتی ہوئی پیچیدگی۔',
  'A humanoid robot sacrifices the efficiency of specialized designs for the flexibility to handle diverse tasks.':
    'ایک ہیومنائڈ روبوٹ متنوع کاموں کو سنبھالنے کی لچک کے لیے خصوصی ڈیزائنوں کی کارکردگی کو قربان کرتا ہے۔',

  // ==================== Technical Terms (Keep in English or Transliterate) ====================
  'AI': 'AI',
  'ROS2': 'ROS2',
  'Robot Operating System': 'Robot Operating System (روبوٹ آپریٹنگ سسٹم)',
  'Digital Twin': 'Digital Twin (ڈیجیٹل جڑواں)',
  'Vision-Language-Action': 'Vision-Language-Action',
  'VLA': 'VLA',
  'Gazebo': 'Gazebo',
  'Isaac Sim': 'Isaac Sim',
  'Python': 'Python',
  'C++': 'C++',
  'Docker': 'Docker',
  'Linux': 'Linux',
  'Ubuntu': 'Ubuntu',

  // ==================== Button/Action Text ====================
  'Read More': 'مزید پڑھیں',
  'Learn More': 'مزید جانیں',
  'Get Started': 'شروع کریں',
  'Continue Reading': 'پڑھنا جاری رکھیں',
  'Previous': 'پچھلا',
  'Next': 'اگلا',
  'Back': 'واپس',
  'Home': 'ہوم',
  'Download': 'ڈاؤن لوڈ',
  'Install': 'انسٹال کریں',
  'Run': 'چلائیں',
  'Build': 'بنائیں',
  'Test': 'ٹیسٹ کریں',

  // ==================== Common MDX Elements ====================
  'Code Example': 'کوڈ کی مثال',
  'Terminal Output': 'ٹرمینل آؤٹ پٹ',
  'Configuration': 'کنفیگریشن',
  'Installation': 'انسٹالیشن',
  'Setup': 'سیٹ اپ',
  'Usage': 'استعمال',
  'API Reference': 'API حوالہ',
  'Documentation': 'دستاویزات',

  // ==================== Status/State Messages ====================
  'In Progress': 'جاری ہے',
  'Completed': 'مکمل',
  'Not Started': 'شروع نہیں ہوا',
  'Optional': 'اختیاری',
  'Required': 'ضروری',
  'Recommended': 'تجویز کردہ',
  'Advanced': 'جدید',
  'Beginner': 'ابتدائی',
  'Intermediate': 'درمیانی',

  // ==================== Time/Date ====================
  'Today': 'آج',
  'Yesterday': 'کل',
  'Tomorrow': 'کل',
  'Week': 'ہفتہ',
  'Month': 'مہینہ',
  'Year': 'سال',
  'Updated': 'اپ ڈیٹ شدہ',
  'Published': 'شائع شدہ',
  'Last Modified': 'آخری ترمیم',

  // ==================== Categories ====================
  'Category': 'زمرہ',
  'Tags': 'ٹیگز',
  'Topics': 'موضوعات',
  'Authors': 'مصنفین',
  'Contributors': 'شراکت دار',
  'Version': 'ورژن',
  'License': 'لائسنس',

  // ==================== Additional Humanoid Landscape Content ====================
  'The Current Landscape: Major Players': 'موجودہ منظرنامہ: بڑے کھلاڑی',
  'Industry Leaders': 'صنعت کے رہنما',
  'The humanoid robotics field has evolved from academic research to serious commercial ventures:':
    'ہیومنائڈ روبوٹکس کا میدان تعلیمی تحقیق سے سنجیدہ تجارتی منصوبوں میں تبدیل ہو گیا ہے:',
  'Focus': 'توجہ',
  'Legacy Pioneers': 'روایتی پیش رو',
  'Research & advanced mobility': 'تحقیق اور جدید نقل و حرکت',
  'AI-First Startups': 'AI-پہلے اسٹارٹ اپس',
  'Foundation model integration': 'بنیادی ماڈل انضمام',
  'Tech Giants': 'ٹیکنالوجی کے دیو',
  'Manufacturing & consumer scale': 'مینوفیکچرنگ اور کنزیومر پیمانے',
  'Logistics-Focused': 'لاجسٹکس پر مرکوز',
  'Warehouse automation': 'گودام آٹومیشن',

  // Platform Deep Dives
  'Platform Deep Dives': 'پلیٹ فارم کی گہری تحقیق',
  'Boston Dynamics Atlas': 'بوسٹن ڈائنامکس اٹلس',
  'Technical Specifications': 'تکنیکی تفصیلات',
  'Height': 'قد',
  'Weight': 'وزن',
  'Degrees of Freedom': 'آزادی کے درجات',
  'Sensors': 'سینسرز',
  'Actuators': 'ایکچویٹرز',
  'Key Capabilities': 'کلیدی صلاحیتیں',
  'Design Philosophy': 'ڈیزائن کا فلسفہ',
  'Current Status': 'موجودہ حیثیت',
  'Stereo vision, depth sensors, IMU': 'سٹیریو ویژن، گہرائی سینسر، IMU',
  'Hydraulic and electric hybrid': 'ہائیڈرولک اور الیکٹرک ہائبرڈ',
  'Atlas represents the': 'اٹلس کی نمائندگی کرتا ہے',
  'dynamic mobility': 'متحرک نقل و حرکت',
  'approach, prioritizing agility and athletic performance.':
    'نقطہ نظر، چستی اور کھیلوں کی کارکردگی کو ترجیح دیتے ہوئے۔',
  'Primarily research platform; transitioning to commercial applications in warehouse logistics.':
    'بنیادی طور پر تحقیقی پلیٹ فارم؛ گودام لاجسٹکس میں تجارتی ایپلیکیشنز میں منتقلی۔',

  // Tesla Optimus
  'Tesla Optimus (Bot Gen 2)': 'ٹیسلا آپٹیمس (بوٹ جنریشن 2)',
  'including dexterous hands': 'ماہر ہاتھوں سمیت',

  // Section headings with emojis
  '🎯 Learning Objectives': '🎯 سیکھنے کے مقاصد',
  '📚 Prerequisites': '📚 پیشگی ضروریات',
  '💡 Key Takeaways': '💡 کلیدی نکات',
  '⚠️ Important': '⚠️ اہم',
  '🔔 Note': '🔔 نوٹ',
};

/**
 * Helper function to get all translation keys
 */
export function getAllTranslationKeys(): string[] {
  return Object.keys(contentTranslations);
}

/**
 * Helper function to check if a translation exists
 */
export function hasTranslation(text: string): boolean {
  return text in contentTranslations;
}
